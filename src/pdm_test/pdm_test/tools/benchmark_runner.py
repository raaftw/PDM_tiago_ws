import argparse
import os
import re
import shutil
import subprocess
import sys
import time
import json
import signal
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


def run_launch_and_record(run_dir: Path, mode: str, world: str, map_path: str | None,
                          nav2_params: str | None, nav2_mode: str, use_sim_time: bool, use_rviz: bool) -> int:
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

    print('Launching benchmark...')
    print(' '.join(cmd))
    
    # Clean up any lingering processes from previous runs
    cleanup_processes()
    
    start = time.time()
    start_iso = datetime.now().isoformat(timespec='seconds')

    # Ensure parent of bag exists
    run_dir.mkdir(parents=True, exist_ok=True)

    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
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

    meta = {
        'mode': mode,
        'world': world,
        'map': map_path,
        'nav2_params': nav2_params,
        'nav2_mode': nav2_mode,
        'use_sim_time': use_sim_time,
        'use_rviz': use_rviz,
        'start_time_iso': start_iso,
        'end_time_iso': end_iso,
        'duration_s': round(end - start, 3),
        'bag_dir': str(bag_dir),
        'cwd': str(Path.cwd()),
        'git': git_info(Path.cwd()),
        'launch_return_code': ret,
        'launch_cmd': cmd,
    }
    write_json(run_dir / 'meta.json', meta)

    return ret


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
    return result.returncode


def main():
    parser = argparse.ArgumentParser(description='Benchmark run wrapper: organize runs, record bag, analyze results.')
    parser.add_argument('--mode', choices=['mpc', 'nav2'], required=True, help='Which stack to run')
    parser.add_argument('--world', default='cafe', help='World name (e.g., cafe, walls_blocks)')
    parser.add_argument('--map', dest='map_path', default=None, help='Override map yaml path')
    parser.add_argument('--nav2-params', default=None, help='Nav2 params yaml (if mode=nav2)')
    parser.add_argument('--nav2-mode', default='tiago_public', choices=['tiago_public', 'direct'],
                        help='Nav2 mode: tiago_public (external TIAGo sim) or direct (nav2_bringup)')
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
        'root': str(root),
    }
    write_json(run_dir / 'config.json', init_cfg)

    print(f'Run directory: {run_dir}')

    ret = run_launch_and_record(
        run_dir=run_dir,
        mode=mode,
        world=args.world,
        map_path=args.map_path,
        nav2_params=args.nav2_params,
        nav2_mode=args.nav2_mode,
        use_sim_time=args.use_sim_time,
        use_rviz=use_rviz,
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
