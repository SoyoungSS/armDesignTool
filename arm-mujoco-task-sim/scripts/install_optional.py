"""Install a matching, hash-checked optional profile in a CPython 3.12 project venv."""
import argparse
from importlib.metadata import PackageNotFoundError, version
import json
from pathlib import Path
import subprocess
import sys

from environment import file_hash, platform_tag


PROFILES = {
    "model": (),
    "usd": ("usd-core",),
    "camera": ("glfw", "pyopengl"),
    "generator-3.3.7": ("usd-core", "mujoco", "glfw", "pyopengl", "absl-py",
                        "etils", "fsspec", "importlib-resources", "zipp", "typing-extensions"),
}


def installed(name):
    try:
        return version(name)
    except PackageNotFoundError:
        return None


def install(profile):
    if sys.prefix == sys.base_prefix:
        raise ValueError("Run this script with the Python inside the project virtual environment.")
    assets = Path(__file__).resolve().parents[1] / "assets"
    optional = assets / "wheels-optional"
    tag = platform_tag()
    if tag not in ("cp312-linux_x86_64", "cp312-win_amd64"):
        raise ValueError("Use the bundled x64 CPython 3.12 for Windows or Linux.")
    files = {}
    if PROFILES[profile]:
        records = json.loads((optional / "bundle.json").read_text(encoding="utf8"))[tag]
        for package in PROFILES[profile]:
            entry = records[package]
            files[optional / entry["file"]] = entry["sha256"]
    if profile == "generator-3.3.7":
        if installed("mujoco-core") or installed("mujoco-studio") or installed("mujoco") not in (None, "3.3.7"):
            raise ValueError("Create a fresh venv for the MuJoCo 3.3.7 generator profile.")
    if profile in ("model", "generator-3.3.7"):
        folder = assets / "wheels" / tag
        base = json.loads((folder / "bundle.json").read_text(encoding="utf8"))
        for name, digest in base["wheels"].items():
            if name.startswith(("numpy-", "pillow-")):
                files[folder / name] = digest
    for path, digest in files.items():
        if file_hash(path) != digest:
            raise ValueError("Damaged wheel: " + str(path))
    pip = [sys.executable, "-I", "-m", "pip", "--isolated", "--disable-pip-version-check"]
    subprocess.run([*pip, "install", "--no-index", "--no-cache-dir", "--only-binary=:all:",
                    *map(str, files)], check=True)
    subprocess.run([*pip, "check"], check=True)
    if profile in ("usd", "generator-3.3.7"):
        subprocess.run([sys.executable, "-I", "-c",
                        "from pxr import Usd, UsdPhysics; assert Usd.Stage.CreateInMemory()"], check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", choices=PROFILES)
    try:
        install(parser.parse_args().profile)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(str(error))
