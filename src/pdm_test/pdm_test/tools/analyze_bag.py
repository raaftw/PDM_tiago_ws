import argparse
import os
import json
from typing import Dict, List, Tuple

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as e:
    rosbag2_py = None


def ns_to_s(ns: int) -> float:
    return ns / 1e9


def percentile(a: List[float], p: float) -> float:
    if not a:
        return float('nan')
    return float(np.percentile(np.asarray(a, dtype=float), p))


def series_stats(values: List[float]) -> Dict[str, float]:
    if not values:
        return {
            'count': 0,
            'mean': float('nan'),
            'min': float('nan'),
            'max': float('nan'),
            'p95': float('nan'),
            'p99': float('nan'),
        }
    arr = np.asarray(values, dtype=float)
    return {
        'count': int(arr.size),
        'mean': float(arr.mean()),
        'min': float(arr.min()),
        'max': float(arr.max()),
        'p95': float(np.percentile(arr, 95)),
        'p99': float(np.percentile(arr, 99)),
    }


def read_bag_series(bag_path: str,
                    topic_whitelist: List[str]) -> Dict[str, List[Tuple[float, object]]]:
    if rosbag2_py is None:
        raise RuntimeError('rosbag2_py is not available in this environment.')

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='',
                                                   output_serialization_format='')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Build type map
    topics_and_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics_and_types}

    # Prepare deserializers for whitelisted topics that exist in bag
    topic_types: Dict[str, str] = {}
    msg_types: Dict[str, object] = {}
    for topic in topic_whitelist:
        if topic in type_map:
            topic_types[topic] = type_map[topic]
            try:
                msg_types[topic] = get_message(type_map[topic])
            except Exception:
                # Skip if we cannot import the type
                pass

    series: Dict[str, List[Tuple[float, object]]] = {t: [] for t in topic_types.keys()}

    t0_ns = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in series:
            continue
        if t0_ns is None:
            t0_ns = t_ns
        try:
            msg_cls = msg_types.get(topic)
            if msg_cls is None:
                continue
            msg = deserialize_message(data, msg_cls)
            series[topic].append((ns_to_s(t_ns - t0_ns), msg))
        except Exception:
            # Skip messages we fail to deserialize
            continue

    return series


def extract_float_series(series: List[Tuple[float, object]], attr: str = 'data') -> Tuple[List[float], List[float]]:
    t: List[float] = []
    v: List[float] = []
    for ts, msg in series:
        try:
            v.append(float(getattr(msg, attr)))
            t.append(ts)
        except Exception:
            continue
    return t, v


def extract_odom_positions(series: List[Tuple[float, object]]) -> Tuple[List[float], List[float], List[float]]:
    t: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    for ts, msg in series:
        try:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            xs.append(x)
            ys.append(y)
            t.append(ts)
        except Exception:
            continue
    return t, xs, ys


def extract_cmd_vel(series: List[Tuple[float, object]]) -> Tuple[List[float], List[float], List[float]]:
    t: List[float] = []
    vxs: List[float] = []
    wzs: List[float] = []
    for ts, msg in series:
        try:
            vxs.append(float(msg.linear.x))
            wzs.append(float(msg.angular.z))
            t.append(ts)
        except Exception:
            continue
    return t, vxs, wzs


def path_length(xs: List[float], ys: List[float]) -> float:
    if len(xs) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        d = (dx*dx + dy*dy)**0.5
        total += d
    return float(total)


def numerical_derivative(t: List[float], x: List[float]) -> List[float]:
    if len(t) < 2:
        return []
    a: List[float] = []
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        if dt <= 0:
            a.append(0.0)
            continue
        a.append((x[i] - x[i-1]) / dt)
    return a


def write_csv(path: str, header: List[str], rows: List[List[float]]):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write(','.join(header) + '\n')
        for r in rows:
            f.write(','.join(str(x) for x in r) + '\n')


def analyze_bag(bag_path: str, out_prefix: str) -> Dict[str, object]:
    topics = [
        '/metrics/min_obstacle_distance',
        '/metrics/path_lateral_error',
        '/ground_truth_odom',
        '/cmd_vel',
    ]

    series = read_bag_series(bag_path, topics)

    # Obstacle distance
    t_obs, obs_vals = extract_float_series(series.get('/metrics/min_obstacle_distance', []))
    obs_stats = series_stats(obs_vals)

    # Path error
    t_err, err_vals = extract_float_series(series.get('/metrics/path_lateral_error', []))
    err_stats = series_stats(err_vals)

    # Odom path length and duration
    t_odom, xs, ys = extract_odom_positions(series.get('/ground_truth_odom', []))
    length = path_length(xs, ys)
    duration = (t_odom[-1] - t_odom[0]) if len(t_odom) >= 2 else (t_obs[-1] - t_obs[0]) if len(t_obs) >= 2 else float('nan')

    # Smoothness from cmd_vel
    t_cmd, vxs, wzs = extract_cmd_vel(series.get('/cmd_vel', []))
    acc = numerical_derivative(t_cmd, vxs)
    jerk = numerical_derivative(t_cmd[1:], acc) if len(t_cmd) > 2 else []

    acc_stats = series_stats(acc)
    jerk_stats = series_stats(jerk)

    # Write timeseries CSVs
    if t_obs:
        write_csv(f"{out_prefix}_obstacle_distance.csv", ['t', 'min_obstacle_distance'], list(zip(t_obs, obs_vals)))
    if t_err:
        write_csv(f"{out_prefix}_path_error.csv", ['t', 'path_lateral_error'], list(zip(t_err, err_vals)))
    if t_odom:
        write_csv(f"{out_prefix}_odom_xy.csv", ['t', 'x', 'y'], list(zip(t_odom, xs, ys)))
    if t_cmd:
        write_csv(f"{out_prefix}_cmd_vel.csv", ['t', 'linear_x', 'angular_z'], list(zip(t_cmd, vxs, wzs)))

    summary = {
        'bag_path': bag_path,
        'duration_s': float(duration),
        'path_length_m': float(length),
        'min_obstacle_distance': obs_stats,
        'path_lateral_error': err_stats,
        'acc_linear_x': acc_stats,
        'jerk_linear_x': jerk_stats,
    }

    with open(f"{out_prefix}_summary.json", 'w') as f:
        json.dump(summary, f, indent=2)

    return summary


def main():
    parser = argparse.ArgumentParser(description='Analyze rosbag2 benchmark metrics and produce CSV + JSON summary.')
    parser.add_argument('--bag', required=True, help='Path to rosbag2 directory (folder containing metadata.yaml)')
    parser.add_argument('--out', required=False, default=None, help='Output file prefix (without extension). Default: <bag>_analysis/<basename>')
    args = parser.parse_args()

    bag = os.path.abspath(args.bag)
    if not os.path.isdir(bag) or not os.path.exists(os.path.join(bag, 'metadata.yaml')):
        raise SystemExit(f'Not a rosbag2 directory: {bag}')

    if args.out:
        out_prefix = os.path.abspath(args.out)
    else:
        base = os.path.basename(os.path.normpath(bag))
        out_dir = os.path.join(os.path.dirname(bag), f'{base}_analysis')
        os.makedirs(out_dir, exist_ok=True)
        out_prefix = os.path.join(out_dir, base)

    summary = analyze_bag(bag, out_prefix)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
