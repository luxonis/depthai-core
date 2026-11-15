#!/usr/bin/env python3
"""Measure Linux and host CPU during camera_transfer_test's payload interval."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import time


DEVICE_SNAPSHOT = '''import glob,json,os,time
processes={}
for path in glob.glob("/proc/[0-9]*/stat"):
 try:
  raw=open(path).read(); name=raw[raw.index("(")+1:raw.rindex(")")]; fields=raw[raw.rindex(")")+2:].split()
  if name.startswith("depthai-device"):
   processes[path.split("/")[2]]={"ticks":int(fields[11])+int(fields[12]),"start":fields[19]}
 except (OSError,ValueError): pass
print(json.dumps({"monotonic":time.monotonic(),"hz":os.sysconf("SC_CLK_TCK"),"cpu":list(map(int,open("/proc/stat").readline().split()[1:9])),"processes":processes}))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', required=True, type=Path, help='Built camera_transfer_test')
    parser.add_argument('--device', required=True, help='DepthAI device name, e.g. USB port path or IP address')
    parser.add_argument('--protocol', required=True, choices=['usb', 'tcpip'])
    parser.add_argument('--ssh-target', required=True, help='SSH destination for the same reserved device')
    parser.add_argument('--ssh-option', action='append', default=[], help='Additional ssh -o option; may be repeated')
    parser.add_argument('--fps', type=float, default=2)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', required=True, type=Path, help='Prefix for log, CSV, NV12 and CPU JSON')
    args = parser.parse_args()
    ssh = ['ssh', '-n', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10']
    for option in args.ssh_option:
        ssh += ['-o', option]
    ssh += ['--', args.ssh_target]

    def snapshot(pid):
        raw = Path(f'/proc/{pid}/stat').read_text()
        fields = raw[raw.rindex(')') + 2:].split()
        host = {'monotonic': time.monotonic(), 'ticks': int(fields[11]) + int(fields[12]), 'hz': os.sysconf('SC_CLK_TCK')}
        device = json.loads(subprocess.check_output(ssh + ['python3 -c ' + shlex.quote(DEVICE_SNAPSHOT)], text=True, timeout=15))
        return {'device': device, 'host': host}

    environment = dict(os.environ, DEPTHAI_DEVICE_NAME_LIST=args.device, DEPTHAI_PROTOCOL=args.protocol,
                       DEPTHAI_TRANSFER_FPS=str(args.fps), DEPTHAI_TRANSFER_SECONDS=str(args.seconds),
                       DEPTHAI_TRANSFER_MIN_GBPS='0', DEPTHAI_TRANSFER_OUTPUT=str(args.output))
    start = end = None
    report = {'protocol': args.protocol, 'fps': args.fps, 'cpu_unit': 'percent of one CPU core'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix('.log').open('w') as log:
        run = subprocess.Popen([str(args.executable.resolve())], env=environment, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in run.stdout:
                print(line, end='', flush=True)
                log.write(line)
                log.flush()
                if line.startswith('measurement_start '):
                    protocol = 'usb' if args.protocol == 'usb' else 'tcp'
                    if not line.startswith(f'measurement_start protocol={protocol} '):
                        raise RuntimeError('Camera connected through a different transport')
                    start = snapshot(run.pid)
                elif line.startswith('frames='):
                    end = snapshot(run.pid)
                    report['payload_measurement'] = dict(field.split('=', 1) for field in line.split())
            report['exit_code'] = run.wait()
        finally:
            if run.poll() is None:
                run.terminate()
                try:
                    run.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    run.kill()
                    run.wait()
    report.update(start=start, end=end)
    if start and end:
        before, after = start['device'], end['device']
        interval = after['monotonic'] - before['monotonic']
        ticks = [b - a for a, b in zip(before['cpu'], after['cpu'])]
        report['device_cpu'] = {
            'seconds': interval,
            'busy_core_percent': sum(ticks[i] for i in (0, 1, 2, 5, 6, 7)) / before['hz'] / interval * 100,
            'system_core_percent': ticks[2] / before['hz'] / interval * 100,
            'irq_core_percent': (ticks[5] + ticks[6]) / before['hz'] / interval * 100,
        }
        stable_firmware = before['processes'] and before['processes'].keys() == after['processes'].keys() and all(
            after['processes'][pid]['start'] == previous['start'] for pid, previous in before['processes'].items())
        if stable_firmware:
            firmware_ticks = sum(after['processes'][pid]['ticks'] - previous['ticks'] for pid, previous in before['processes'].items())
            report['firmware_core_percent'] = firmware_ticks / before['hz'] / interval * 100
        else:
            report['firmware_core_percent'] = None
            report['measurement_error'] = 'Firmware process was missing or changed during the CPU measurement'
        a, b = start['host'], end['host']
        report['host_core_percent'] = (b['ticks'] - a['ticks']) / a['hz'] / (b['monotonic'] - a['monotonic']) * 100
    args.output.with_suffix('.cpu.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key not in ('start', 'end')}, indent=2))
    return report['exit_code'] or (0 if start and end and 'measurement_error' not in report else 1)


if __name__ == '__main__':
    raise SystemExit(main())
