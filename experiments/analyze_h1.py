#!/usr/bin/env python3
"""Regenerate the H1 statistics (docs/evaluation.md Sections 5, 6 and 8)
from a learning-curve run's `curve_data.json`.

    python3 experiments/analyze_h1.py results/h1_learning_curve/curve_data.json

Prints a Markdown report and writes it next to the input as
`h1_analysis.md`. See src/opportunity_rl/h1_analysis.py for the
statistical scheme (independent units + exact sign test).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from opportunity_rl.h1_analysis import CurveData, format_report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("curve_data", type=Path, help="path to curve_data.json")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="where to write the report (default: h1_analysis.md beside the input)",
    )
    args = parser.parse_args()

    data = CurveData.from_json(json.loads(args.curve_data.read_text()))
    report = format_report(data)
    print(report)

    out = args.out or args.curve_data.with_name("h1_analysis.md")
    out.write_text(report + "\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
