"""Solve an offline dataset without ROS or configuration deployment."""
import argparse
import json
from pathlib import Path
from .handeye_workflow import atomic_write_json, solve_dataset


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    if options.input.resolve() == options.output.resolve():
        parser.error('output must not overwrite input dataset')
    report = solve_dataset(json.loads(options.input.read_text(encoding='utf-8')))
    atomic_write_json(options.output, report)
    print(json.dumps({'passed': report['passed'], 'selected_method': report['selected_method']}))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
