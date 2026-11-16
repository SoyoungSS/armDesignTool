"""Render another view or pose from a previously generated run."""
import argparse
from pathlib import Path

from arm_sim.model import read_input
from arm_sim.preview import render_model


def render(run, output, pose='initial', view='iso'):
    model = read_input(run / 'report' / 'normalized_model.json')
    render_model(model, run / 'meshes', output, pose, view)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pose', default='initial', help='initial, geometric_zero or an acceptance pose name')
    parser.add_argument('--view', choices=['iso', 'front', 'side'], default='iso')
    args = parser.parse_args()
    render(args.run, args.output, args.pose, args.view)
