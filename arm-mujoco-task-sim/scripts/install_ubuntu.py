"""Bootstrap the bundled x86-64 CPython and offline wheel set on Ubuntu."""
import argparse
import hashlib
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile


def portable_python(assets, cache):
    directory = assets / "python-linux-x86_64"
    digest, name = (directory / "SHA256SUMS").read_text().split()
    if Path(name).name != name or len(digest) != 64:
        raise ValueError("Invalid interpreter manifest")
    cache.mkdir(parents=True, exist_ok=True)
    target = cache / ("cpython-" + digest[:16])
    exe = target / "bin" / "python3.12"
    if exe.is_file():
        return exe
    archive = directory / name
    checksum = hashlib.sha256()
    with archive.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
    if checksum.hexdigest() != digest:
        raise ValueError("Interpreter archive failed SHA256 verification: " + str(archive))
    with tempfile.TemporaryDirectory(dir=cache) as work:
        subprocess.run(["tar", "-xzf", str(archive), "-C", work], check=True)
        try:
            (Path(work) / "python").rename(target)
        except OSError:
            if not exe.is_file():  # another installer may have published the same archive
                raise
    return exe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["web", "native", "headless"], required=True)
    parser.add_argument("--venv", type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != "linux" or platform.machine() != "x86_64":
        parser.error("Use this bundle on x86-64 Linux. See references/install-api.md for other platforms.")
    libc = os.confstr("CS_GNU_LIBC_VERSION").split()
    if libc[0] != "glibc" or tuple(map(int, libc[1].split(".")[:2])) < (2, 31):
        parser.error("This Ubuntu bundle targets glibc 2.31 or later.")
    scripts = Path(__file__).resolve().parent
    cache = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "arm-mujoco-task-sim"
    exe = portable_python(scripts.parent / "assets", cache)
    subprocess.run([str(exe), "-I", str(scripts / "environment.py"), "install",
                    "--mode", args.mode, "--venv", str(args.venv)], check=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
