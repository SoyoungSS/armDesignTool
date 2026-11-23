"""Bake the generated MJCF and measured task poses for the offline browser viewer.

Model:  python prepare-viewer.py --scene robot.xml --output model.json
Task:   add --replay replay.jsonl --result result.json
"""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil

if os.name == "nt" and os.environ.get("ARM_SIM_SYSTEM_MSVC") == "1":
    ctypes.WinDLL(str(Path(os.environ["SystemRoot"]) / "System32/msvcp140.dll"))
import mujoco as mj
import numpy as np


def numbers(array):
    return np.asarray(array, dtype=float).round(6).tolist()


def geometry(model):
    meshes = {}
    geoms = []
    for i in range(model.ngeom):
        if model.geom_group[i] >= 3:
            continue
        kind = mj.mjtGeom(int(model.geom_type[i])).name.removeprefix("mjGEOM_").lower()
        material = model.geom_matid[i]
        rgba = model.geom_rgba[i] if material < 0 else model.mat_rgba[material]
        if rgba[3] <= 0:
            continue
        geom = dict(name=model.geom(i).name, body=int(model.geom_bodyid[i]), kind=kind,
                    size=numbers(model.geom_size[i]), pos=numbers(model.geom_pos[i]),
                    quat=numbers(model.geom_quat[i]), rgba=numbers(rgba))
        if kind == "mesh":
            j = int(model.geom_dataid[i])
            geom["mesh"] = str(j)
            if str(j) not in meshes:
                start, count = int(model.mesh_vertadr[j]), int(model.mesh_vertnum[j])
                first, length = int(model.mesh_faceadr[j]), int(model.mesh_facenum[j])
                meshes[str(j)] = dict(vertices=numbers(model.mesh_vert[start:start+count].ravel()),
                                      faces=model.mesh_face[first:first+length].ravel().tolist())
        elif kind not in ("plane", "box", "sphere", "ellipsoid", "cylinder", "capsule"):
            raise ValueError("Unsupported scene geometry: " + kind)
        geoms.append(geom)
    return geoms, meshes


def body_poses(model, data):
    mj.mj_kinematics(model, data)
    return numbers(np.column_stack((data.xpos, data.xquat)).ravel())


def build(scene, output, replay=None, result_file=None, validation=None):
    model = mj.MjModel.from_xml_path(str(scene.resolve()))
    data = mj.MjData(model)
    geoms, meshes = geometry(model)
    frames = []
    spec = dict(version=1, geoms=geoms, meshes=meshes,
                source_sha256=hashlib.sha256(scene.read_bytes()).hexdigest())
    if replay:
        result = json.loads(result_file.read_text(encoding="utf-8-sig"))
        if result.get("scene_sha256") and spec["source_sha256"] != result["scene_sha256"]:
            raise ValueError(f"Replay scene does not match the recorded result: {scene}")
        with replay.open(encoding="utf-8-sig") as stream:
            for line in stream:
                row = json.loads(line)
                data.qpos[:] = row["qpos"]
                frames.append(dict(t=round(row["time_s"], 5), cycle=row["cycle"],
                                   phase=row["phase"], bodies=body_poses(model, data)))
        spec["result"] = {key: result.get(key) for key in
                          ("status", "reason", "steps", "duration_s", "evaluation", "collision")}
        if not frames:
            raise ValueError("Replay has no samples")
    else:
        key = mj.mj_name2id(model, mj.mjtObj.mjOBJ_KEY, "initial")
        if key >= 0:
            mj.mj_resetDataKeyframe(model, data, key)
        frames.append(dict(t=0, phase="初始姿态", bodies=body_poses(model, data)))
        mj.mj_resetData(model, data)
        frames.append(dict(t=0, phase="几何零位", bodies=body_poses(model, data)))
        spec["joints"] = [dict(name=model.joint(i).name, limits=numbers(model.jnt_range[i]))
                          for i in range(model.njnt)]
        if validation:
            review = json.loads(validation.read_text(encoding="utf8"))["collision_review"]
            spec["collision"] = dict(overall=review["status"], poses={
                name: evidence["status"] for name, evidence in review["named_pose_results"].items()
            })
    spec["frames"] = frames
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".partial")
    temporary.write_text(json.dumps(spec, ensure_ascii=False, allow_nan=False,
                                    separators=(",", ":")), encoding="utf-8")
    temporary.replace(output)
    print(f"{output}: {len(geoms)} visual geometries, {len(frames)} poses")


def prepare_defaults(results, kind):
    for arm in ("worksheet", "fr3"):
        dest = results / "viewer" / arm
        if kind in ("both", "models"):
            parent = results / "models" / arm
            attempt = json.loads((parent / "latest_attempt.json").read_text(encoding="utf8"))
            run = parent / "runs" / Path(attempt["run_directory"].replace("\\", "/")).name
            manifest = json.loads((run / "report/manifest.json").read_text(encoding="utf8"))
            scene = run / manifest["targets"]["mujoco"]["entry"].replace("\\", "/")
            build(scene, dest / "model.json", validation=run / "report/validation.json")
            shutil.copyfile(run / "report/validation.json", dest / "validation.json")
            shutil.copyfile(run / "preview.png", dest / "preview.png")
        if kind in ("both", "sorting"):
            parent = results / "sorting" / arm
            for report in sorted((parent / "runs").glob("*/reports/result.json"), reverse=True):
                run = report.parents[1]
                replay = run / "data/replay.jsonl"
                outcome = json.loads(report.read_text(encoding="utf8"))
                if outcome["status"] == "passed" and replay.is_file():
                    contract = json.loads((run / "input/task_contract.json").read_text(encoding="utf8"))
                    if contract.get("mode") != "headless":
                        continue
                    build(run / "input" / contract["scene"], dest / "sorting.json", replay, report)
                    shutil.copyfile(report, dest / "result.json")
                    break
            else:
                raise ValueError(f"No accepted headless sorting replay in {parent}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--scene", type=Path)
    inputs.add_argument("--results-root", type=Path, help="publish completed default examples")
    parser.add_argument("--kind", choices=["models","sorting","both"], default="both")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--result", type=Path)
    options = parser.parse_args()
    if bool(options.replay) != bool(options.result):
        parser.error("--result and --replay must be supplied together")
    if options.results_root:
        prepare_defaults(options.results_root, options.kind)
    else:
        if not options.output:
            parser.error("--scene requires --output")
        build(options.scene, options.output, options.replay, options.result)

