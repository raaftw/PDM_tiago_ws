import argparse
import os
import re
import shutil
import subprocess
import sys
import time
import json
import signal
import threading
from datetime import datetime
from pathlib import Path

# Run manager for consistent, numbered benchmark folders.
# - Creates runs like <root>/run_001_mpc, <root>/run_002_nav2
# - Records bag to <run_dir>/bag
# - After exit, analyzes bag into <run_dir>/summary/* and writes meta.json
#
# Usage examples:
#   python3 -m pdm_test.tools.benchmark_runner --mode mpc --world walls_blocks
#   python3 -m pdm_test.tools.benchmark_runner --mode nav2 --world walls_blocks \
#       --nav2-params /path/to/nav2_params.yaml


def next_run_dir(root: Path, mode: str) -> Path:
    mode_root = root / mode
    mode_root.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(r"^run_(\d{3})$")
    max_n = 0
    for child in mode_root.iterdir():
        if child.is_dir():
            m = pattern.match(child.name)
            if m:
                try:
                    n = int(m.group(1))
                    max_n = max(max_n, n)
                except ValueError:
                    pass
    run_dir = mode_root / f"run_{max_n+1:03d}"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def write_json(path: Path, obj: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        json.dump(obj, f, indent=2)


def git_info(cwd: Path) -> dict:
    try:
        # Get short commit hash and branch if in a git repo
        commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(cwd), stderr=subprocess.DEVNULL).decode().strip()
        branch = subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD'], cwd=str(cwd), stderr=subprocess.DEVNULL).decode().strip()
        return {'commit': commit, 'branch': branch}
    except Exception:
        return {}


def cleanup_processes():
    """Kill any lingering simulation/nav processes before starting a new run."""
    procs = ['gzserver', 'gzclient', 'move_group', 'play_motion2', 'rviz2', 'nav2', 
             'mpc_controller', 'global_planner', 'map_server']
    for proc in procs:
        try:
            subprocess.run(['pkill', '-9', proc], stderr=subprocess.DEVNULL, timeout=2)
        except Exception:
            pass
    time.sleep(0.5)  # Brief pause for cleanup


class GoalTimingMonitor:
    """Monitor stdout/stderr for goal set and completion messages."""
    def __init__(self, mode: str):
        self.mode = mode
        self.goal_set_time = None
        self.goal_complete_time = None
        self.rrt_computation_time = None  # For MPC: time to compute RRT* path
        self.ros_time_offset = None  # Will store first ROS timestamp seen
        
        if mode == 'mpc':
            # For MPC, goal is published by goal_publisher node
            self.goal_set_pattern = re.compile(r"goal_publisher.*Publishing goal now")
            # RRT* path computation time (for reference)
            self.rrt_path_found_pattern = re.compile(r"rrt_star_planner_node.*Path found with")
            # MPC driving done when it calls hand motion service (heading aligned)
            self.goal_complete_pattern = re.compile(r"Heading aligned! Calling hand motion service")
        else:  # nav2
            # Nav2: goal published by goal_publisher "Publishing goal now" (same as MPC)
            self.goal_set_pattern = re.compile(r"goal_publisher.*Publishing goal now")
            # Nav2 completes when bt_navigator reaches the goal
            self.goal_complete_pattern = re.compile(r"(Reached goal|Goal succeeded|Goal reached)")
            self.rrt_path_found_pattern = None
        
        # Pattern to extract ROS timestamp: [INFO] [1234567890.123456789]
        self.ros_time_pattern = re.compile(r"\[INFO\]\s+\[(\d+\.\d+)\]")
    
    def process_line(self, line: str):
        """Check if line contains goal-related events and extract ROS timestamp."""
        # Extract ROS timestamp from the line if present
        ros_time_match = self.ros_time_pattern.search(line)
        if ros_time_match:
            ros_timestamp = float(ros_time_match.group(1))
            # Store offset from first ROS timestamp (simulation start)
            if self.ros_time_offset is None:
                self.ros_time_offset = ros_timestamp
        
        # Capture goal publication time (from goal_publisher for both MPC and Nav2)
        if self.goal_set_time is None and self.goal_set_pattern.search(line):
            if ros_time_match:
                self.goal_set_time = float(ros_time_match.group(1))
                print(f"[TIMING] Goal published at ROS time {self.goal_set_time}")
            else:
                self.goal_set_time = time.time()
                print(f"[TIMING] Goal published at wall clock {self.goal_set_time}")
        
        # For MPC: capture RRT* path computation time
        if self.mode == 'mpc' and self.rrt_path_found_pattern and self.rrt_computation_time is None:
            if self.rrt_path_found_pattern.search(line) and ros_time_match and self.goal_set_time:
                rrt_path_time = float(ros_time_match.group(1))
                self.rrt_computation_time = rrt_path_time - self.goal_set_time
                print(f"[TIMING] RRT* path computed in {self.rrt_computation_time:.3f}s")
        
        # Capture goal completion time
        if self.goal_complete_pattern.search(line):
            if ros_time_match:
                self.goal_complete_time = float(ros_time_match.group(1))
                print(f"[TIMING] Driving complete at ROS time {self.goal_complete_time}")
            else:
                self.goal_complete_time = time.time()
                print(f"[TIMING] Driving complete at wall clock {self.goal_complete_time}")
    
    def get_goal_duration(self) -> dict:
        """Return timing info as dict."""
        result = {
            'goal_set_time': self.goal_set_time,
            'goal_complete_time': self.goal_complete_time,
            'goal_achievement_duration_s': None,
            'rrt_computation_time_s': self.rrt_computation_time,
        }
        if self.goal_set_time and self.goal_complete_time:
            result['goal_achievement_duration_s'] = round(self.goal_complete_time - self.goal_set_time, 3)
        return result


def run_launch_and_record(run_dir: Path, mode: str, world: str, map_path: str | None,
                          nav2_params: str | None, nav2_mode: str, use_sim_time: bool, use_rviz: bool,
                          goal_location: str | None) -> tuple[int, dict]:
    bag_dir = run_dir / 'bag'
    bag_prefix = str(bag_dir)

    # Build command for our toggle launch
    cmd = [
        'ros2', 'launch', 'pdm_test', 'benchmark_switch.launch.py',
        f'use_nav2:={"true" if mode == "nav2" else "false"}',
        f'world_name:={world}',
        f'bag_prefix:={bag_prefix}',
        f'use_sim_time:={"true" if use_sim_time else "false"}',
        f'use_rviz:={"true" if use_rviz else "false"}',
    ]

    if mode == 'nav2':
        cmd.append(f'nav2_mode:={nav2_mode}')
    if map_path:
        cmd.append(f'map:={map_path}')
    if mode == 'nav2' and nav2_params:
        cmd.append(f'nav2_params:={nav2_params}')
    if goal_location:
        cmd.append(f'goal_location:={goal_location}')

    print('Launching benchmark...')
    print(' '.join(cmd))
    
    # Clean up any lingering processes from previous runs
    cleanup_processes()
    
    start = time.time()
    start_iso = datetime.now().isoformat(timespec='seconds')

    # Ensure parent of bag exists
    run_dir.mkdir(parents=True, exist_ok=True)

    # Create timing monitor (mode-aware)
    timing_monitor = GoalTimingMonitor(mode)
    
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid, stdout=subprocess.PIPE, 
                            stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1)
    
    # Thread to monitor output for timing markers
    def monitor_output():
        try:
            for line in proc.stdout:
                print(line, end='')  # Print to console
                timing_monitor.process_line(line)
        except Exception:
            pass
    
    monitor_thread = threading.Thread(target=monitor_output, daemon=True)
    monitor_thread.start()
    
    ret = 0
    try:
        ret = proc.wait()
    except KeyboardInterrupt:
        print('\nInterrupted. Shutting down cleanly...')
        # Kill entire process group to catch all children
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            ret = proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print('Timeout waiting for clean shutdown, forcing kill...')
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                ret = proc.wait(timeout=3)
            except Exception:
                ret = -9
        except Exception:
            # Fallback if process group kill fails
            proc.kill()
            ret = -9
        finally:
            # Final cleanup pass
            cleanup_processes()

    end = time.time()
    end_iso = datetime.now().isoformat(timespec='seconds')
    
    # Get goal timing
    goal_timing = timing_monitor.get_goal_duration()

    meta = {
        'mode': mode,
        'world': world,
        'map': map_path,
        'nav2_params': nav2_params,
        'nav2_mode': nav2_mode,
        'use_sim_time': use_sim_time,
        'use_rviz': use_rviz,
        'goal_location': goal_location,
        'start_time_iso': start_iso,
        'end_time_iso': end_iso,
        'duration_s': round(end - start, 3),
        'goal_timing': goal_timing,
        'bag_dir': str(bag_dir),
        'cwd': str(Path.cwd()),
        'git': git_info(Path.cwd()),
        'launch_return_code': ret,
        'launch_cmd': cmd,
    }
    write_json(run_dir / 'meta.json', meta)
    
    # Print timing summary
    if goal_timing['goal_achievement_duration_s'] is not None:
        print(f"\n[SUMMARY] Goal achievement time: {goal_timing['goal_achievement_duration_s']:.3f}s")
    else:
        print("\n[SUMMARY] Goal achievement timing not captured")

    return ret, goal_timing


def analyze(run_dir: Path):
    bag_dir = run_dir / 'bag'
    if not (bag_dir / 'metadata.yaml').exists():
        # If benchmark_switch appended timestamp, pick first matching dir
        # Fallback: find any subdir with metadata.yaml
        candidates = list(run_dir.glob('bag*'))
        for c in candidates:
            if (c / 'metadata.yaml').exists():
                bag_dir = c
                break
    if not (bag_dir / 'metadata.yaml').exists():
        print(f'No rosbag2 found under {run_dir}. Skipping analysis.', file=sys.stderr)
        return 2

    out_prefix = run_dir / 'summary' / 'run'
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        '-m', 'pdm_test.tools.analyze_bag',
        '--bag', str(bag_dir),
        '--out', str(out_prefix)
    ]
    print('Analyzing bag...')
    print(' '.join(cmd))
    result = subprocess.run(cmd)
    
    # Add goal timing from meta.json to the summary
    meta_path = run_dir / 'meta.json'
    summary_path = run_dir / 'summary' / 'run_summary.json'
    
    if meta_path.exists() and summary_path.exists():
        try:
            with meta_path.open('r') as f:
                meta = json.load(f)
            with summary_path.open('r') as f:
                summary = json.load(f)
            
            # Add only goal achievement duration to summary
            if 'goal_timing' in meta and meta['goal_timing'].get('goal_achievement_duration_s') is not None:
                summary['goal_achievement_duration_s'] = meta['goal_timing']['goal_achievement_duration_s']
            
            # Determine task success/failure
            goal_achieved = 'goal_achievement_duration_s' in summary and summary['goal_achievement_duration_s'] is not None
            collision_detected = summary.get('collision_detected', False)
            
            failure_reasons = []
            if not goal_achieved:
                failure_reasons.append('goal_not_reached')
            if collision_detected:
                failure_reasons.append('collision_detected')
            
            task_achieved = goal_achieved and not collision_detected
            task_failed = not task_achieved
            
            summary['task_achieved'] = task_achieved
            summary['task_failed'] = task_failed
            summary['failure_reasons'] = failure_reasons
            
            # Write updated summary
            with summary_path.open('w') as f:
                json.dump(summary, f, indent=2)
            
            print(f'Task Status: achieved={task_achieved}, failed={task_failed}')
            if failure_reasons:
                print(f'Failure reasons: {", ".join(failure_reasons)}')
        except Exception as e:
            print(f'Warning: Could not merge goal timing into summary: {e}')
    
    return result.returncode


def main():
    parser = argparse.ArgumentParser(description='Benchmark run wrapper: organize runs, record bag, analyze results.')
    parser.add_argument('--mode', choices=['mpc', 'nav2'], required=True, help='Which stack to run')
    parser.add_argument('--world', default='cafe', help='World name (e.g., cafe, walls_blocks)')
    parser.add_argument('--map', dest='map_path', default=None, help='Override map yaml path')
    parser.add_argument('--nav2-params', default=None, help='Nav2 params yaml (if mode=nav2)')
    parser.add_argument('--nav2-mode', default='tiago_public', choices=['tiago_public', 'direct'],
                        help='Nav2 mode: tiago_public (external TIAGo sim) or direct (nav2_bringup)')
    parser.add_argument('--goal-location', default=None, help='Goal location (center, corner_1, corner_2, corner_3, corner_4)')
    parser.add_argument('--root', default='runs', help='Root directory to store runs')
    parser.add_argument('--use-sim-time', action='store_true', help='Pass use_sim_time true')
    parser.add_argument('--no-rviz', action='store_true', help='Disable RViz in Nav2 branch')

    args = parser.parse_args()

    root = Path(args.root).resolve()
    mode = args.mode
    use_rviz = not args.no_rviz

    run_dir = next_run_dir(root, mode)

    # Persist initial config
    init_cfg = {
        'mode': mode,
        'world': args.world,
        'map': args.map_path,
        'nav2_params': args.nav2_params,
        'nav2_mode': args.nav2_mode,
        'goal_location': args.goal_location,
        'root': str(root),
    }
    write_json(run_dir / 'config.json', init_cfg)

    print(f'Run directory: {run_dir}')

    ret, goal_timing = run_launch_and_record(
        run_dir=run_dir,
        mode=mode,
        world=args.world,
        map_path=args.map_path,
        nav2_params=args.nav2_params,
        nav2_mode=args.nav2_mode,
        use_sim_time=args.use_sim_time,
        use_rviz=use_rviz,
        goal_location=args.goal_location,
    )

    # Always try analysis after launch exits
    analyze(run_dir)

    # Make/update a convenient symlink to latest
    try:
        mode_root = root / mode
        latest = mode_root / 'latest'
        if latest.exists() or latest.is_symlink():
            latest.unlink()
        latest.symlink_to(run_dir)
    except Exception:
        pass

    print(f'Finished. See {run_dir}/summary and {run_dir}/meta.json')
    sys.exit(ret if isinstance(ret, int) else 0)


if __name__ == '__main__':
    main()
