import argparse
import os
import json
import math
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


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw angle in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


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


def extract_odom_full(series: List[Tuple[float, object]]) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Extract time, x, y, and yaw from odometry."""
    t: List[float] = []
    xs: List[float] = []
    ys: List[float] = []
    yaws: List[float] = []
    for ts, msg in series:
        try:
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            qx = float(msg.pose.pose.orientation.x)
            qy = float(msg.pose.pose.orientation.y)
            qz = float(msg.pose.pose.orientation.z)
            qw = float(msg.pose.pose.orientation.w)
            yaw = quaternion_to_yaw(qx, qy, qz, qw)
            xs.append(x)
            ys.append(y)
            yaws.append(yaw)
            t.append(ts)
        except Exception:
            continue
    return t, xs, ys, yaws


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
        '/goal_pose',  # For MPC goal timing
        '/_action/navigate_to_pose/send_goal',  # For Nav2 goal timing
        '/_action/navigate_to_pose/status',  # For Nav2 completion timing
    ]

    series = read_bag_series(bag_path, topics)

    # Obstacle distance
    t_obs, obs_vals = extract_float_series(series.get('/metrics/min_obstacle_distance', []))
    obs_stats = series_stats(obs_vals)

    # Path error
    t_err, err_vals = extract_float_series(series.get('/metrics/path_lateral_error', []))
    err_stats = series_stats(err_vals)

    # Odom path length and duration
    t_odom, xs, ys, yaws = extract_odom_full(series.get('/ground_truth_odom', []))
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

    # Detect collision: minimum distance below 0.1m threshold
    collision_threshold = 0.1  # meters
    collision_detected = False
    min_distance_value = float('inf')
    if obs_vals:
        min_distance_value = min(obs_vals)
        collision_detected = min_distance_value < collision_threshold

    # Goal achievement timing - read from meta.json which now has ROS timestamps from benchmark_runner
    goal_achievement_duration_s = None
    rrt_computation_time_s = None
    goal_reached_time_ros = None
    
    # Extract goal position for accuracy metrics
    goal_x = None
    goal_y = None
    goal_theta = None
    
    # Try to load goal timing from meta.json
    if xs and ys:
        bag_dir = os.path.dirname(bag_path)
        run_dir = os.path.dirname(bag_dir) if 'bag' in os.path.basename(bag_dir) else bag_dir
        meta_path = os.path.join(run_dir, 'meta.json')
        
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r') as f:
                    meta = json.load(f)
                    if 'goal_timing' in meta:
                        if 'goal_achievement_duration_s' in meta['goal_timing']:
                            # The meta.json now contains ROS timestamps from benchmark_runner
                            # This is the actual time from goal publication to goal achievement in ROS time
                            goal_achievement_duration_s = meta['goal_timing']['goal_achievement_duration_s']
                        if 'rrt_computation_time_s' in meta['goal_timing']:
                            # RRT* path computation time (MPC only)
                            rrt_computation_time_s = meta['goal_timing']['rrt_computation_time_s']
            except Exception:
                pass

    # Goal accuracy metrics (compare final state to goal)
    goal_position_error = None
    goal_heading_error_rad = None
    goal_heading_error_deg = None
    final_x = None
    final_y = None
    final_heading = None
    
    if xs and ys and yaws:
        final_x = float(xs[-1])
        final_y = float(ys[-1])
        final_heading = float(yaws[-1])
        
        # Try to load goal from meta.json
        bag_dir = os.path.dirname(bag_path)
        run_dir = os.path.dirname(bag_dir) if 'bag' in os.path.basename(bag_dir) else bag_dir
        meta_path = os.path.join(run_dir, 'meta.json')
        
        if os.path.exists(meta_path):
            try:
                with open(meta_path, 'r') as f:
                    meta = json.load(f)
                
                # Get goal from launch command (check for goal_location or goal_x/y/theta)
                if 'launch_cmd' in meta:
                    cmd_str = ' '.join(meta['launch_cmd'])
                    # Extract goal_x, goal_y, goal_theta from command
                    import re
                    x_match = re.search(r'goal_x:=([\-\d.]+)', cmd_str)
                    y_match = re.search(r'goal_y:=([\-\d.]+)', cmd_str)
                    theta_match = re.search(r'goal_theta:=([\-\d.]+)', cmd_str)
                    
                    if x_match and y_match and theta_match:
                        goal_x = float(x_match.group(1))
                        goal_y = float(y_match.group(1))
                        goal_theta = float(theta_match.group(1))
                    else:
                        # Try to parse goal_location
                        loc_match = re.search(r'goal_location:=(\w+)', cmd_str)
                        if loc_match:
                            location = loc_match.group(1)
                            # Predefined locations (same as goal_publisher.py)
                            locations = {
                                'center': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
                                'corner_1': {'x': 2.5, 'y': 1.8, 'theta': 1.57},
                                'corner_2': {'x': 2.5, 'y': -1.8, 'theta': -1.57},
                                'corner_3': {'x': -2.5, 'y': 1.8, 'theta': 1.57},
                                'corner_4': {'x': -2.5, 'y': -1.8, 'theta': -1.57},
                            }
                            if location in locations:
                                goal_x = locations[location]['x']
                                goal_y = locations[location]['y']
                                goal_theta = locations[location]['theta']
                
                if goal_x is not None and goal_y is not None and goal_theta is not None:
                    # Compute position error (Euclidean distance)
                    dx = final_x - goal_x
                    dy = final_y - goal_y
                    goal_position_error = math.sqrt(dx*dx + dy*dy)
                    
                    # Compute heading error (normalized to [-pi, pi])
                    heading_diff = normalize_angle(final_heading - goal_theta)
                    goal_heading_error_rad = abs(heading_diff)
                    goal_heading_error_deg = math.degrees(goal_heading_error_rad)
            except Exception as e:
                pass  # Silently skip if meta.json not readable

    summary = {
        'path_length_m': float(length),
        'min_obstacle_distance': obs_stats,
        'collision_detected': collision_detected,
        'min_distance_value': float(min_distance_value) if min_distance_value != float('inf') else None,
        'final_position': {'x': final_x, 'y': final_y} if final_x is not None else None,
        'final_heading_rad': final_heading,
        'goal_position_error_m': goal_position_error,
        'goal_heading_error_rad': goal_heading_error_rad,
        'goal_heading_error_deg': goal_heading_error_deg,
        'goal_achievement_duration_s': goal_achievement_duration_s,
        'rrt_computation_time_s': rrt_computation_time_s,
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
