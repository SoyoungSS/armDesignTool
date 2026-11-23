"""Probe a task interpreter, prepare a wheel bundle, or install it without network."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import sysconfig
import tempfile
import venv


def platform_tag():
    abi = sys.implementation.cache_tag.replace('cpython-', 'cp')
    if sysconfig.get_config_var('Py_GIL_DISABLED'):
        abi += 't'
    return abi + '-' + sysconfig.get_platform().lower().replace('-', '_')


def file_hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def seal_bundle(folder):
    wheels = sorted(folder.glob('*.whl'))
    if not wheels:
        raise ValueError('No wheels found in ' + str(folder))
    manifest = {'platform': platform_tag(), 'source': 'https://pypi.org/simple',
                'wheels': {p.name: file_hash(p) for p in wheels}}
    (folder/'bundle.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


def pack(args):
    destination = args.bundle.resolve()
    if destination.exists():
        raise ValueError('Choose a new bundle directory: ' + str(destination))
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=destination.parent) as work:
        folder = Path(work)/'wheels'
        package = 'mujoco-core' if args.mode == 'headless' else 'mujoco-studio'
        subprocess.run([sys.executable, '-m', 'pip', '--isolated', '--disable-pip-version-check',
                        'download', '--only-binary=:all:', '--index-url', 'https://pypi.org/simple',
                        '--dest', str(folder), package + '==' + args.version, 'numpy>=2,<3',
                        'matplotlib'], check=True)
        seal_bundle(folder)
        folder.rename(destination)
    print(destination)


def install(args):
    folder = args.bundle.resolve()
    manifest = json.loads((folder/'bundle.json').read_text(encoding='utf-8'))
    if manifest['platform'] != platform_tag():
        raise ValueError(f'Bundle needs {manifest["platform"]}. This Python is {platform_tag()}.')
    # Platform-neutral wheels ship once, shared by the two checked-in bundles.
    shared = set(manifest.get('shared', []))
    inventory = set(manifest['wheels'])
    if not shared <= inventory or any(Path(n).name != n for n in inventory):
        raise ValueError('Invalid wheel inventory in bundle.json')
    files = sorted(folder.glob('*.whl'))
    if not inventory or {p.name for p in files} != inventory - shared:
        raise ValueError('Wheel inventory differs from bundle.json')
    files += [folder.parent/'common'/name for name in sorted(shared)]
    for path in files:
        if file_hash(path) != manifest['wheels'][path.name]:
            raise ValueError('Damaged wheel: ' + path.name)
    dest = args.venv.resolve()
    if not (dest/'pyvenv.cfg').is_file():
        if dest.exists() and any(dest.iterdir()):
            raise ValueError('Choose a new project environment directory')
        venv.EnvBuilder(with_pip=True).create(dest)
    exe = dest/('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    check = 'import sys,sysconfig; print(sys.implementation.cache_tag, sysconfig.get_platform(), bool(sysconfig.get_config_var("Py_GIL_DISABLED")))'
    if subprocess.check_output([str(exe), '-I', '-c', check]) != subprocess.check_output([sys.executable, '-I', '-c', check]):
        raise ValueError('Existing venv has another ABI. Choose a new --venv directory.')
    # Only local wheel paths reach pip. Resolve all transitive dependencies from the bundle.
    subprocess.run([str(exe), '-I', '-m', 'pip', '--isolated', '--disable-pip-version-check',
                    'install', '--no-index', '--no-cache-dir', '--only-binary=:all:',
                    *map(str, files)], check=True)
    subprocess.run([str(exe), '-m', 'pip', 'check'], check=True)
    subprocess.run([str(exe), str(Path(__file__).resolve()), 'probe', '--mode', args.mode], check=True)
    print(str(exe))


def probe(args):
    from runtime import mj, np
    model = mj.MjModel.from_xml_string('<mujoco><worldbody><body><joint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
    data = mj.MjData(model)
    mj.mj_step(model, data)
    mj.mj_fullM(model, data, np.empty((model.nv, model.nv)))
    if args.mode == 'web':
        from mujoco.studio import launch_web
        from mujoco.studio.web import web_server
        from viewer_ui import viewer_plugins
    elif args.mode == 'native':
        from mujoco.studio import launch_native, native_viewer
        from viewer_ui import viewer_plugins
    plot_python = os.environ.get('ARM_SIM_ANALYSIS_PYTHON') or sys.executable
    plot_check = ('import matplotlib; from matplotlib.backends.backend_agg import FigureCanvasAgg; '
                  'from matplotlib.figure import Figure; FigureCanvasAgg(Figure(figsize=(1,1))).draw(); '
                  'print(matplotlib.__version__)')
    plot_version = subprocess.check_output([plot_python, '-c', plot_check], text=True).strip()
    print(json.dumps(dict(python=sys.executable, platform=platform_tag(), mode=args.mode,
                          mujoco=mj.__version__, numpy=np.__version__, matplotlib=plot_version,
                          plot_python=plot_python,
                          check='physics step, dense mass matrix, selected imports, plot canvas'), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    for name in ('probe', 'pack', 'install'):
        sub = commands.add_parser(name)
        sub.add_argument('--mode', choices=['web', 'native', 'headless'], required=True)
        if name == 'pack':
            sub.add_argument('--bundle', type=Path, required=True)
            sub.add_argument('--version', required=True)
        if name == 'install':
            sub.add_argument('--bundle', type=Path, default=Path(__file__).resolve().parents[1] /
                             'assets'/'wheels'/platform_tag())
            sub.add_argument('--venv', type=Path, required=True)
    args = parser.parse_args()
    {'probe': probe, 'pack': pack, 'install': install}[args.action](args)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, ImportError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
