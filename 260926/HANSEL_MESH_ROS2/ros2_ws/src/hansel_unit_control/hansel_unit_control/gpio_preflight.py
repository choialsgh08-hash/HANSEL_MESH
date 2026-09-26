"""Read-only configuration and local pigpiod connectivity check."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .rpi_gpio import GPIO_DEFAULTS, PIN_KEYS, checked, connect_local, validate_options


def check_environment(root, params_file):
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError('Install python3-yaml in the ROS Python environment') from exc
    source = Path(root) / 'ros2_ws/src/hansel_unit_control/hansel_unit_control'
    installed = Path(__file__).resolve().parent
    for name in ('hardware.py', 'core.py', 'unit_controller.py', 'rpi_gpio.py', 'gpio_preflight.py'):
        if not (source / name).is_file() or (source / name).read_bytes() != (installed / name).read_bytes():
            raise RuntimeError(f'Stale installed controller: {name}. Rebuild this workspace.')
    data = yaml.safe_load(Path(params_file).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or set(data) != {'/**'} or not isinstance(data['/**'], dict):
        raise ValueError('Use a /**: ros__parameters: YAML mapping')
    params = data['/**'].get('ros__parameters')
    if not isinstance(params, dict):
        raise ValueError('Missing ros__parameters mapping')
    unknown = [key for key in params if key.startswith('gpio_') and key not in GPIO_DEFAULTS]
    if unknown:
        raise ValueError('Unknown GPIO parameters: ' + ', '.join(unknown))
    options = validate_options({key.removeprefix('gpio_'): value
                                for key, value in params.items() if key in GPIO_DEFAULTS})
    pigpio, pi = connect_local(options['pigpio_port'])
    try:
        version = checked(pi.get_pigpio_version())
        revision = checked(pi.get_hardware_revision())
        for key in PIN_KEYS:
            pin = options[key]
            mode = checked(pi.get_mode(pin))
            if mode not in (pigpio.INPUT, pigpio.OUTPUT):
                raise RuntimeError(f'BCM{pin} uses an alternate function; check overlays first')
    finally:
        pi.stop()
    print(f'[OK] Python={sys.executable}; pigpiod={version}; board_revision=0x{revision:x}')
    print('[OK] Local daemon/configuration checked without motor or servo output.')
    print('[INFO] pigpio does not provide exclusive pin ownership; use one controller per Pi.')
    return version


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True)
    parser.add_argument('--params', required=True)
    args = parser.parse_args(argv)
    try:
        check_environment(args.root, args.params)
    except Exception as exc:
        print(f'[ERROR] GPIO preflight: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
