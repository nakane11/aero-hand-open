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
and main.cpp) which match it at every address in common. Some registers on
this HLS3606M deviate from the generic table's documented VALUE RANGE (e.g.
current_limit's real max is 1023, not the table's 511 -- see
REG_CURRENT_LIMIT's clamp in firmware/main/src/main.cpp); the per-unit scale
factors below are the ones already used/verified elsewhere in this codebase
(aero_open_sdk.aero_hand) where such a register is also read via the
sync-read path (temperature, current, speed).
"""

from __future__ import annotations
import argparse
import sys

from aero_open_sdk.aero_hand import AeroHand

BAUD_RATE_BPS = {
    0: 1000000, 1: 500000, 2: 250000, 3: 128000,
    4: 115200, 5: 76800, 6: 57600, 7: 38400,
}
MODE_LABELS = {0: "position", 1: "constant_speed", 2: "pwm_open_loop", 3: "step_servo"}
MOVING_LABELS = {0: "stopped", 1: "moving"}

# bit index -> label, for the servo_status (addr 65) error bitmask
STATUS_BITS = {
    0: "voltage_error",
    1: "encoder_error",
    2: "temperature_error",
    3: "current_error",
    5: "load_error",
}

# One entry per named register:
#   addr, size (bytes), area ("eeprom"/"sram"/"sram_ro")
#   scale: raw (or sign-magnitude-decoded raw) * scale = displayed value, in `unit`
#   sign_bit: if set, raw is decoded as sign-magnitude with this bit as the sign flag
#             and the bits below it as the magnitude (Feetech's convention for several
#             velocity/torque/offset registers) before `scale` is applied
#   enum: optional {raw: label} lookup that overrides scale/unit entirely
#   bitmask: if true, displayed as a hex value plus any STATUS_BITS names that are set
REGISTERS = {
    "id":                    dict(addr=5,  size=1, area="eeprom"),
    "baud_rate":             dict(addr=6,  size=1, area="eeprom", enum=BAUD_RATE_BPS),
    "second_id":             dict(addr=7,  size=1, area="eeprom"),
    "min_angle_limit":       dict(addr=9,  size=2, area="eeprom", scale=0.087890625, unit="deg"),
    "max_angle_limit":       dict(addr=11, size=2, area="eeprom", scale=0.087890625, unit="deg"),
    "max_temperature_limit": dict(addr=13, size=1, area="eeprom", scale=1, unit="degC"),
    "max_voltage_limit":     dict(addr=14, size=1, area="eeprom", scale=0.1, unit="V"),
    "min_voltage_limit":     dict(addr=15, size=1, area="eeprom", scale=0.1, unit="V"),
    "max_torque_limit":      dict(addr=16, size=2, area="eeprom", scale=0.1, unit="%"),
    "cw_dead_zone":          dict(addr=26, size=1, area="eeprom", scale=0.087890625, unit="deg"),
    "ccw_dead_zone":         dict(addr=27, size=1, area="eeprom", scale=0.087890625, unit="deg"),
    "current_limit":         dict(addr=28, size=2, area="eeprom", scale=6.5, unit="mA"),
    "position_offset":       dict(addr=31, size=2, area="eeprom", scale=0.087890625, unit="deg", sign_bit=11),
    "mode":                  dict(addr=33, size=1, area="eeprom", enum=MODE_LABELS),
    "torque_enable":         dict(addr=40, size=1, area="sram"),
    "acceleration":          dict(addr=41, size=1, area="sram", scale=8.7890625, unit="deg/s^2"),
    "goal_position":         dict(addr=42, size=2, area="sram", scale=0.087890625, unit="deg"),
    "goal_torque":           dict(addr=44, size=2, area="sram", scale=0.1, unit="%", sign_bit=10),
    "goal_speed":            dict(addr=46, size=2, area="sram", scale=0.732, unit="RPM", sign_bit=15),
    "torque_limit":          dict(addr=48, size=2, area="sram", scale=0.1, unit="%"),
    "present_position":      dict(addr=56, size=2, area="sram_ro", scale=0.087890625, unit="deg"),
    "present_speed":         dict(addr=58, size=2, area="sram_ro", scale=0.732, unit="RPM", sign_bit=15),
    "present_load":          dict(addr=60, size=2, area="sram_ro", scale=0.1, unit="%", sign_bit=15),
    "present_voltage":       dict(addr=62, size=1, area="sram_ro", scale=0.1, unit="V"),
    "present_temperature":   dict(addr=63, size=1, area="sram_ro", scale=1, unit="degC"),
    "servo_status":          dict(addr=65, size=1, area="sram_ro", bitmask=True),
    "moving":                dict(addr=66, size=1, area="sram_ro", enum=MOVING_LABELS),
    "present_current":       dict(addr=69, size=2, area="sram_ro", scale=6.5, unit="mA", sign_bit=15),
}


def _decode_sign_magnitude(raw: int, sign_bit: int) -> int:
    """Same convention as firmware's decode_signmag15(): bit `sign_bit` is the sign,
    the bits below it are the magnitude. E.g. present_current=32771 (0x8003) with
    sign_bit=15 -> magnitude=3, negative -> -3 (then scaled to mA by the caller)."""
    magnitude = raw & ((1 << sign_bit) - 1)
    return -magnitude if (raw & (1 << sign_bit)) else magnitude


def format_value(raw: int, info: dict | None) -> str:
    """Render a raw register value using its conversion info (None => show raw)."""
    if info is None:
        return str(raw)
    enum = info.get("enum")
    if enum is not None:
        return enum.get(raw, f"unknown({raw})")
    if info.get("bitmask"):
        active = [name for bit, name in STATUS_BITS.items() if raw & (1 << bit)]
        return f"0x{raw:02X}" + (f" [{', '.join(active)}]" if active else " [ok]")
    sign_bit = info.get("sign_bit")
    value = _decode_sign_magnitude(raw, sign_bit) if sign_bit is not None else raw
    scale = info.get("scale")
    if scale is None:
        return str(value)
    converted = round(value * scale, 4)
    unit = info.get("unit", "")
    return f"{converted}{(' ' + unit) if unit else ''}"


def format_formula(info: dict) -> str:
    """One-line description of how a register's raw value maps to the displayed value."""
    enum = info.get("enum")
    if enum is not None:
        return "enum: " + ", ".join(f"{k}={v}" for k, v in sorted(enum.items()))
    if info.get("bitmask"):
        bits = ", ".join(f"bit{b}={name}" for b, name in sorted(STATUS_BITS.items()))
        return f"bitmask (0=ok): {bits}"
    parts = []
    sign_bit = info.get("sign_bit")
    if sign_bit is not None:
        parts.append(f"sign-magnitude (bit{sign_bit}=sign)")
    scale = info.get("scale")
    unit = info.get("unit", "")
    if scale is not None:
        parts.append(f"raw * {scale} = {unit}" if unit else f"raw * {scale}")
    else:
        parts.append("raw (no conversion)")
    return "; ".join(parts)


def _resolve_target(args) -> tuple[int, int, str, dict | None]:
    """Returns (addr, size, area, info) from either --name or --addr/--size."""
    if args.name is not None:
        if args.name not in REGISTERS:
            names = ", ".join(sorted(REGISTERS))
            raise SystemExit(f"Unknown --name '{args.name}'. Known names: {names}")
        info = REGISTERS[args.name]
        return info["addr"], info["size"], info["area"], info
    if args.addr is None:
        raise SystemExit("Specify either --name or --addr")
    size = args.size or 1
    area = "eeprom" if args.addr < 40 else "unknown"
    return args.addr, size, area, None


def cmd_list_params(args):
    print(f"{'name':<22}{'addr':>6}{'size':>6}  {'area':<9}conversion")
    for name, info in sorted(REGISTERS.items(), key=lambda kv: kv[1]["addr"]):
        print(f"{name:<22}{info['addr']:>6}{info['size']:>6}  {info['area']:<9}{format_formula(info)}")


def cmd_get(args):
    hand = AeroHand(port=args.port)
    try:
        addr, size, _area, info = _resolve_target(args)
        ids = [args.id] if args.id is not None else list(range(7))
        for servo_id in ids:
            try:
                raw = hand.read_register(servo_id, addr, size)
                print(f"servo {servo_id}: addr={addr} size={size} value={format_value(raw, info)} (raw={raw})")
            except Exception as e:
                print(f"servo {servo_id}: ERROR: {e}", file=sys.stderr)
    finally:
        hand.close()


def cmd_dump(args):
    """Read every known register for one servo ID (or all 7 if --id is omitted)."""
    hand = AeroHand(port=args.port)
    try:
        ids = [args.id] if args.id is not None else list(range(7))
        items = sorted(REGISTERS.items(), key=lambda kv: kv[1]["addr"])
        for servo_id in ids:
            print(f"=== servo {servo_id} ===")
            print(f"{'name':<22}{'addr':>6}{'size':>6}  {'area':<9}value")
            for name, info in items:
                addr, size, area = info["addr"], info["size"], info["area"]
                try:
                    raw = hand.read_register(servo_id, addr, size)
                    print(f"{name:<22}{addr:>6}{size:>6}  {area:<9}{format_value(raw, info)}")
                except Exception as e:
                    print(f"{name:<22}{addr:>6}{size:>6}  {area:<9}ERROR: {e}", file=sys.stderr)
    finally:
        hand.close()


def cmd_set(args):
    if args.id is None:
        raise SystemExit("--id is required for 'set' (writing all servos at once is not supported)")
    addr, size, area, info = _resolve_target(args)
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
        print(
            f"servo {args.id}: addr={addr} size={size} wrote={args.value} "
            f"readback={format_value(readback, info)} (raw={readback})"
        )
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

    p_list = sub.add_parser("list-params", help="List known named registers and their unit conversions")
    p_list.set_defaults(func=cmd_list_params)

    def add_target_args(p):
        p.add_argument("--name", default=None, help="Named register, see 'list-params'")
        p.add_argument("--addr", type=int, default=None, help="Raw register address (0..255)")
        p.add_argument("--size", type=int, choices=(1, 2), default=None, help="Raw register size in bytes")

    p_get = sub.add_parser("get", help="Read a register (all 7 servos if --id is omitted)")
    add_target_args(p_get)
    p_get.add_argument("--id", type=int, default=None, help="Servo bus ID (0..253)")
    p_get.set_defaults(func=cmd_get)

    p_dump = sub.add_parser("dump", help="Read every known register for one servo (all 7 if --id is omitted)")
    p_dump.add_argument("--id", type=int, default=None, help="Servo bus ID (0..253); all 7 if omitted")
    p_dump.set_defaults(func=cmd_dump)

    p_set = sub.add_parser("set", help="Write a register on one servo")
    add_target_args(p_set)
    p_set.add_argument("--id", type=int, default=None, help="Servo bus ID (0..253), required")
    p_set.add_argument("--value", type=int, required=True, help="Raw value to write (not the converted unit)")
    p_set.add_argument("--force", action="store_true", help="Allow writing to an unconfirmed raw address")
    p_set.set_defaults(func=cmd_set)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
