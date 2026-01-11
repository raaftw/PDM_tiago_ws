#!/usr/bin/env python3
"""
Script to aggregate run summary JSON files into a compact CSV report.
Columns: goal_location, path_length_m, goal_achievement_duration_s,
rrt_computation_time_s, task_achieved, failure_reasons, final_heading_error,
final_location_error, run_name.
"""

import json
import csv
from pathlib import Path
from typing import Dict, List, Any, Optional
import sys


def _read_json(path: Path) -> Optional[dict]:
    try:
        with path.open('r') as f:
            return json.load(f)
    except Exception:
        return None


def get_all_run_summaries(runs_dir: Path) -> List[tuple]:
    """
    Get all run summaries from a directory.
    Returns list of (run_name, data) tuples.
    """
    summaries = []
    
    for run_path in sorted(runs_dir.glob('run_*')):
        if not run_path.is_dir():
            continue
        
        summary_file = run_path / 'summary' / 'run_summary.json'
        if not summary_file.exists():
            continue

        summary_data = _read_json(summary_file)
        if summary_data is None:
            print(f"Warning: Failed to read {summary_file}")
            continue

        meta_file = run_path / 'meta.json'
        meta_data = _read_json(meta_file) if meta_file.exists() else None

        # Build compact row using summary + meta (without modifying source files)
        goal_location = summary_data.get('goal_location')
        if goal_location is None and meta_data:
            goal_location = meta_data.get('goal_location')

        goal_timing = meta_data.get('goal_timing', {}) if meta_data else {}

        # Prefer values in summary.json, fall back to meta.json timing fields
        goal_duration = summary_data.get('goal_achievement_duration_s')
        if goal_duration is None:
            goal_duration = goal_timing.get('goal_achievement_duration_s')

        rrt_time = summary_data.get('rrt_computation_time_s')
        if rrt_time is None:
            rrt_time = goal_timing.get('rrt_computation_time_s')

        # Heading/location errors: prefer explicit goal_* fields if present
        final_heading_err = summary_data.get('goal_heading_error_rad')
        if final_heading_err is None:
            final_heading_err = summary_data.get('goal_heading_error_deg')
        if final_heading_err is None:
            final_heading_err = summary_data.get('final_heading_error')

        final_location_err = summary_data.get('goal_position_error_m')
        if final_location_err is None:
            final_location_err = summary_data.get('final_location_error')

        row = {
            'run_name': run_path.name,
            'goal_location': goal_location,
            'path_length_m': summary_data.get('path_length_m'),
            'goal_achievement_duration_s': goal_duration,
            'rrt_computation_time_s': rrt_time,
            'task_achieved': summary_data.get('task_achieved'),
            'failure_reasons': summary_data.get('failure_reasons'),
            'final_heading_error': final_heading_err,
            'final_location_error': final_location_err,
        }

        summaries.append((run_path.name, row))
    
    return summaries


def main():
    if len(sys.argv) < 2:
        print("Usage: python summarize_runs.py <mpc|nav2> [output_file]")
        print("Example: python summarize_runs.py mpc")
        print("Example: python summarize_runs.py nav2 runs_summary.csv")
        sys.exit(1)
    
    run_type = sys.argv[1].lower()
    output_file = sys.argv[2] if len(sys.argv) > 2 else f"{run_type}_runs_summary.csv"
    
    if run_type not in ['mpc', 'nav2']:
        print(f"Error: run_type must be 'mpc' or 'nav2', got '{run_type}'")
        sys.exit(1)
    
    # Get the runs directory
    runs_dir = Path(__file__).parent / 'runs' / run_type
    
    if not runs_dir.exists():
        print(f"Error: Directory {runs_dir} does not exist")
        sys.exit(1)
    
    # Collect all summaries
    summaries = get_all_run_summaries(runs_dir)
    
    if not summaries:
        print(f"No run summaries found in {runs_dir}")
        sys.exit(1)
    
    print(f"Found {len(summaries)} runs")
    
    # Fixed field order for compact report
    fields = [
        'goal_location',
        'path_length_m',
        'goal_achievement_duration_s',
        'rrt_computation_time_s',
        'task_achieved',
        'failure_reasons',
        'final_heading_error',
        'final_location_error',
        'run_name',
    ]

    # Normalize failure_reasons to string for CSV readability
    rows = []
    for _, data in summaries:
        row = dict(data)
        fr = row.get('failure_reasons')
        if isinstance(fr, list):
            row['failure_reasons'] = ';'.join(str(x) for x in fr)
        rows.append(row)

    # Write CSV
    output_path = Path(output_file)
    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"✓ CSV written to {output_path}")
    print(f"  Fields: {len(fields)}")
    print(f"  Rows: {len(rows)}")


if __name__ == '__main__':
    main()
