#!/usr/bin/env python3
"""Bounded live acceptance; uses the project's existing sampler, never tunes gains.

Requires ../tools/pid_autotune.py and pyserial. Raw CSV and summary JSON go to --out.
--i-own-the-rig explicitly enables motor motion; continuity only sends X and ?.
"""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import statistics
import sys
import time

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]


def load_tools():
    directory = ROOT.parent / 'tools'
    sys.path.insert(0, str(directory))
    spec = importlib.util.spec_from_file_location('acceptance_tools', directory / 'pid_autotune.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def save_frames(path, frames):
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(['time_s', 'target_rpm', 'actual_rpm', 'out_permille'])
        writer.writerows(frames)


def cadence(frames, duration):
    gaps = [(b[0] - a[0]) * 1000 for a, b in zip(frames, frames[1:])]
    ordered = sorted(gaps)
    return {'frames': len(frames), 'frames_per_second': len(frames) / duration,
            'max_receive_gap_ms': max(gaps, default=0),
            'p99_receive_gap_ms': ordered[min(len(ordered)-1, int(len(ordered)*0.99))] if ordered else None,
            'gaps_over_30ms': sum(x > 30 for x in gaps)}


def raw_send(plant, command):
    plant.ser.write(command.encode('ascii'))
    plant.ser.flush()


def checked_status(plant):
    status = plant.health()
    print('STATUS', json.dumps(status), flush=True)
    return status


def continuity(plant, args, result):
    plant.stop()
    plant._read(1.0)
    # Fresh firmware boot precedes this test. Do not query during the measurement window.
    frames = []
    start = time.monotonic()
    while time.monotonic() - start < args.secs:
        offset = time.monotonic() - start
        chunk, _ = plant._read(min(10.0, args.secs - offset))
        frames.extend((t + offset, a, b, c) for t, a, b, c in chunk)
        print(f'continuity {time.monotonic()-start:.1f}/{args.secs:.0f}s: {len(frames)} frames', flush=True)
    duration = time.monotonic() - start
    save_frames(args.out / 'continuity.csv', frames)
    result['cadence'] = cadence(frames, duration)
    result['status_after_capture'] = checked_status(plant)
    result['pass'] = (result['cadence']['gaps_over_30ms'] == 0 and
                      result['cadence']['frames_per_second'] >= 195 and
                      result['status_after_capture'].get('miss', 0) == 0)


def steps(plant, args, result, tools):
    plant.stop()
    plant._read(1.5)
    result['initial_status'] = checked_status(plant)
    trials = []
    result['trials'] = trials
    for rpm in (80, 150, 200):
        for repeat in range(1, 4):
            print(f'STEP {rpm} rpm, {repeat}/3', flush=True)
            frames = plant.run_step(rpm, 3.0, 1.5)
            save_frames(args.out / f'step_{rpm}_{repeat}.csv', frames)
            metrics = tools.sp.metrics(frames, rpm, 3)
            trial = {'rpm': rpm, 'repeat': repeat, 'metrics_smooth3': metrics,
                     'metrics_raw': tools.sp.metrics(frames, rpm, 1),
                     'cadence': cadence(frames, 3.0), 'status': checked_status(plant)}
            trials.append(trial)
            print(json.dumps(trial), flush=True)
            (args.out / 'partial_steps.json').write_text(json.dumps(trials, indent=2))
    result['pass_steady_error'] = all(abs(t['metrics_smooth3']['error_pct']) < 5 for t in trials)
    result['summary_by_rpm'] = {}
    for rpm in (80, 150, 200):
        selected = [t['metrics_smooth3'] for t in trials if t['rpm'] == rpm]
        summary = {}
        for key in ('rise_ms', 'overshoot_pct', 'error_pct', 'ripple_rpm'):
            values = [m[key] for m in selected if m and m[key] is not None]
            summary[key] = {'min': min(values), 'max': max(values), 'mean': statistics.mean(values)} if values else None
        result['summary_by_rpm'][rpm] = summary


def functional(plant, args, result):
    def command(text, wait=0.3):
        raw_send(plant, text)
        frames, _ = plant._read(wait)
        return frames

    tests = []
    result['tests'] = tests
    def record(name, ok, detail):
        tests.append({'name': name, 'pass': bool(ok), 'detail': detail})
        print(json.dumps(tests[-1]), flush=True)
        if not ok:
            raise RuntimeError('functional check failed: ' + name)

    command('X\n', 1.0)
    for prefix in ('S20', 'S12!', 'S123456789012345678901234567890'):
        command('S80\n', 0.6)
        before = checked_status(plant)
        record('start_before_' + prefix, before.get('tgt') == 80 and before.get('act', 0) > 40, before)
        command(prefix, 0.02)
        frames = command('X\n', 0.5)
        status = checked_status(plant)
        zeros = [f for f in frames if f[1] == 0 and f[3] == 0]
        record('emergency_' + prefix, status.get('tgt') == 0 and status.get('out') == 0 and bool(zeros),
               {'status': status, 'first_zero_frame_receive_ms': zeros[0][0] * 1000 if zeros else None})
        command('?\n', 0.2)
        status = checked_status(plant)
        record('stop_latch_' + prefix, status.get('tgt') == 0 and status.get('out') == 0, status)
    command('S80\n', 0.6)
    command('S-80\n', 1.0)
    status = checked_status(plant)
    record('normal_reverse', status.get('tgt') == -80 and status.get('act', 0) < -40, status)
    command('X\n', 1.0)
    frames = command('A\n', 5.4)
    save_frames(args.out / 'automatic_step.csv', frames)
    active = [f for f in frames if f[1] == 150]
    returned = [f for f in frames if f[0] > 5 and f[1] == 0]
    record('relative_autostep', active and returned and 1.9 < active[0][0] < 2.1,
           {'first_150_s': active[0][0] if active else None,
            'first_return_zero_s': returned[0][0] if returned else None})
    command('A\n', 0.5)
    status = checked_status(plant)
    record('autostep_off_stops', status.get('tgt') == 0 and status.get('out') == 0, status)
    command('S80\n', 0.5)
    command('C\n', 0.5)
    status = checked_status(plant)
    record('calibration_entry_stops', status.get('tgt') == 0 and status.get('out') == 0, status)
    command('C\n', 0.5)
    status = checked_status(plant)
    record('calibration_exit_stays_stopped', status.get('tgt') == 0 and status.get('out') == 0, status)
    command('S80\n', 0.5)
    status = checked_status(plant)
    record('explicit_restart', status.get('tgt') == 80 and status.get('act', 0) > 40, status)
    result['pass'] = all(t['pass'] for t in tests)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['continuity', 'steps', 'functional'], required=True)
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--secs', type=float, default=60)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--i-own-the-rig', action='store_true')
    args = parser.parse_args()
    if args.mode != 'continuity' and not args.i_own_the_rig:
        parser.error('motion tests require --i-own-the-rig (motor free to rotate, serial exclusive)')
    args.out.mkdir(parents=True, exist_ok=True)
    tools = load_tools()
    plant = tools.SerialPlant(args.port, 115200)
    plant.ser.exclusive = True
    result = {'mode': args.mode, 'port': args.port, 'timestamp_basis': 'existing SerialPlant receive-chunk interpolation; not MCU timestamps'}
    try:
        plant.ensure_stream()
        if args.mode == 'continuity':
            continuity(plant, args, result)
        elif args.mode == 'steps':
            steps(plant, args, result, tools)
        else:
            functional(plant, args, result)
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            plant.stop()
            plant._read(0.4)
            result['final_status'] = plant.status()
        finally:
            plant.close()
            (args.out / 'summary.json').write_text(json.dumps(result, indent=2))
            print('SUMMARY', json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
