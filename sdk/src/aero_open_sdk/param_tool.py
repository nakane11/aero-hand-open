#!/usr/bin/env python3
# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Read/write configuration registers (torque limit, angle limits, temperature
readback, etc.) on the Feetech HLS3606M servos inside an Aero Hand.

IMPORTANT: this talks directly to the servo bus over the same USB-serial link
used by the ROS interpolator node. Do not run this while
aero_hand_interpolator_node / aero_hand.launch is running -- two processes
cannot share the serial port, and communication will corrupt for both.

The register addresses below come from Feetech's official communication
protocol memory table for the magnetic-encoding servo family
(https://www.feetechrc.com/letter-of-agreement.html -> "Servo Protocol Memory
Table - Magnetic Encoding Version"), cross-checked against this repository's
own vendored register defines (firmware/main/.pio/libdeps/*/FTServo/src/HLSCL.h
and main.cpp) which match it at every address in common. HLS3606M-specific
deviations from this generic table are still possible; treat any address not
already used elsewhere in this codebase with appropriate caution.
"""

from __future__ import annotations
import argparse
import sys

from aero_open_sdk.aero_hand import AeroHand

# name -> (address, size_in_bytes, area)
# area is "eeprom" (persistent, unlock/lock handled by the firmware) or
# "sram" (volatile, resets on power-cycle) or "sram_ro" (read-only status).
KNOWN_REGISTERS = {
    "id":                  (5,  1, "eeprom"),
    "baud_rate":           (6,  1, "eeprom"),
    "second_id":           (7,  1, "eeprom"),
    "min_angle_limit":     (9,  2, "eeprom"),
    "max_angle_limit":     (11, 2, "eeprom"),
    "max_temperature_limit": (13, 1, "eeprom"),  # deg C, range 0-100
    "max_voltage_limit":   (14, 1, "eeprom"),  # 0.1V units, range 0-254 (0-25.4V)
    "min_voltage_limit":   (15, 1, "eeprom"),  # 0.1V units, range 0-254 (0-25.4V)
    "max_torque_limit":    (16, 2, "eeprom"),  # 0.001 units, range 0-1000 (fraction of stall torque)
    "cw_dead_zone":        (26, 1, "eeprom"),
    "ccw_dead_zone":       (27, 1, "eeprom"),
    "current_limit":       (28, 2, "eeprom"),
    "position_offset":     (31, 2, "eeprom"),
    "mode":                (33, 1, "eeprom"),
    "torque_enable":       (40, 1, "sram"),
    "acceleration":        (41, 1, "sram"),
    "goal_position":       (42, 2, "sram"),
    "goal_torque":         (44, 2, "sram"),
    "goal_speed":          (46, 2, "sram"),
    "torque_limit":        (48, 2, "sram"),
    "present_position":    (56, 2, "sram_ro"),
    "present_speed":       (58, 2, "sram_ro"),
    "present_load":        (60, 2, "sram_ro"),
    "present_voltage":     (62, 1, "sram_ro"),
    "present_temperature": (63, 1, "sram_ro"),
    "servo_status":        (65, 1, "sram_ro"),  # error bits: voltage/encoder/temp/current/load
    "moving":              (66, 1, "sram_ro"),
    "present_current":     (69, 2, "sram_ro"),
}


def _resolve_target(args) -> tuple[int, int, str]:
    """Returns (addr, size, area) from either --name or --addr/--size."""
    if args.name is not None:
        if args.name not in KNOWN_REGISTERS:
            names = ", ".join(sorted(KNOWN_REGISTERS))
            raise SystemExit(f"Unknown --name '{args.name}'. Known names: {names}")
        addr, size, area = KNOWN_REGISTERS[args.name]
        return addr, size, area
    if args.addr is None:
        raise SystemExit("Specify either --name or --addr")
    size = args.size or 1
    area = "eeprom" if args.addr < 40 else "unknown"
    return args.addr, size, area


def cmd_list_params(args):
    print(f"{'name':<20}{'addr':>6}{'size':>6}  area")
    for name, (addr, size, area) in sorted(KNOWN_REGISTERS.items(), key=lambda kv: kv[1][0]):
        print(f"{name:<20}{addr:>6}{size:>6}  {area}")


def cmd_get(args):
    hand = AeroHand(port=args.port)
    try:
        addr, size, _area = _resolve_target(args)
        ids = [args.id] if args.id is not None else list(range(7))
        for servo_id in ids:
            try:
                value = hand.read_register(servo_id, addr, size)
                print(f"servo {servo_id}: addr={addr} size={size} value={value}")
            except Exception as e:
                print(f"servo {servo_id}: ERROR: {e}", file=sys.stderr)
    finally:
        hand.close()


def cmd_set(args):
    if args.id is None:
        raise SystemExit("--id is required for 'set' (writing all servos at once is not supported)")
    addr, size, area = _resolve_target(args)
    if args.name is not None and area == "sram_ro":
        raise SystemExit(f"'{args.name}' is a read-only status register; refusing to write it")
    if args.name is None and not args.force:
        # Raw --addr writes are always unverified against this codebase's register map.
        raise SystemExit(
            "Raw --addr writes are not verified against this codebase's register map. "
            "Verify the address against the official HLS3606M datasheet, then pass --force to proceed."
        )
    if area == "eeprom":
        print(
            f"WARNING: addr {addr} is in the EEPROM (persistent) region. "
            "EEPROM has a limited write-cycle lifetime; avoid writing repeatedly.",
            file=sys.stderr,
        )

    hand = AeroHand(port=args.port)
    try:
        readback = hand.write_register(args.id, addr, args.value, size)
        print(f"servo {args.id}: addr={addr} size={size} wrote={args.value} readback={readback}")
        if readback != args.value:
            print("WARNING: read-back value does not match the written value.", file=sys.stderr)
    finally:
        hand.close()


def build_parser():
    parser = argparse.ArgumentParser(
        description="Read/write Aero Hand servo configuration registers. "
        "Do not run this while aero_hand_interpolator_node / aero_hand.launch is running."
    )
    parser.add_argument("--port", default=None, help="Serial port (default: auto-detect)")
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list-params", help="List known named registers")
    p_list.set_defaults(func=cmd_list_params)

    def add_target_args(p):
        p.add_argument("--name", default=None, help="Named register, see 'list-params'")
        p.add_argument("--addr", type=int, default=None, help="Raw register address (0..255)")
        p.add_argument("--size", type=int, choices=(1, 2), default=None, help="Raw register size in bytes")

    p_get = sub.add_parser("get", help="Read a register (all 7 servos if --id is omitted)")
    add_target_args(p_get)
    p_get.add_argument("--id", type=int, default=None, help="Servo bus ID (0..253)")
    p_get.set_defaults(func=cmd_get)

    p_set = sub.add_parser("set", help="Write a register on one servo")
    add_target_args(p_set)
    p_set.add_argument("--id", type=int, default=None, help="Servo bus ID (0..253), required")
    p_set.add_argument("--value", type=int, required=True, help="Value to write")
    p_set.add_argument("--force", action="store_true", help="Allow writing to an unconfirmed raw address")
    p_set.set_defaults(func=cmd_set)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
