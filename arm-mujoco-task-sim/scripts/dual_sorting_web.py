"""Start the local MuJoCo Studio viewer on an already prepared sorting project."""
import argparse
import json
import os
from pathlib import Path

from dual_sorting import SortingSession
from run_task import launch_viewer


class WebSortingSession(SortingSession):
    def __init__(self, project):
        super().__init__(project, mode="web")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--open-browser", action="store_true")
    parser.add_argument("--ready-file", type=Path, help="publish the live address for the local workbench")
    args = parser.parse_args()
    if not 0 < args.port < 65536:
        parser.error("--port must be in 1..65535")
    session = WebSortingSession(args.project)

    def ready(handle):
        url = f"http://127.0.0.1:{args.port}/"
        print(url, flush=True)
        if args.ready_file:
            args.ready_file.parent.mkdir(parents=True, exist_ok=True)
            temp = args.ready_file.with_suffix(f".{os.getpid()}.tmp")
            temp.write_text(json.dumps(dict(url=url, pid=os.getpid())), encoding="utf8")
            temp.replace(args.ready_file)

    try:
        launch_viewer(session, port=args.port, open_browser=args.open_browser,
                      on_ready=ready, web_idle_s=0)
    finally:
        if args.ready_file and args.ready_file.is_file():
            published = json.loads(args.ready_file.read_text(encoding="utf8"))
            if published.get("pid") == os.getpid():
                args.ready_file.unlink()


if __name__ == "__main__":
    main()

