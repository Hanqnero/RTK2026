#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  Simple Cartesian teleop for meArm.

import argparse

import meArm


def parse_args():
    parser = argparse.ArgumentParser(description="Move the meArm in x/y/z coordinates.")
    parser.add_argument("--block", type=int, default=0, help="PWM servo block to use, 0 to 3.")
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=0x40,
        help="PWM I2C address, e.g. 0x40.",
    )
    parser.add_argument("--step", type=float, default=10.0, help="Nudge size in mm.")
    return parser.parse_args()


def print_help(step):
    print("")
    print("Commands:")
    print("  x+ x-     move left/right by %.1f mm" % step)
    print("  y+ y-     move forward/back by %.1f mm" % step)
    print("  z+ z-     move up/down by %.1f mm" % step)
    print("  goto X Y Z")
    print("  step N    set nudge size to N mm")
    print("  open, close, pos, help, quit")
    print("")


def main():
    args = parse_args()
    step = args.step

    arm = meArm.meArm()
    arm.begin(block=args.block, address=args.address)

    print("meArm XYZ teleop. Current position: %s" % arm.getPos())
    print_help(step)

    while True:
        try:
            line = input("xyz> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("")
            break

        if not line:
            continue

        parts = line.split()
        command = parts[0]
        x, y, z = arm.getPos()

        if command in ("quit", "exit", "q"):
            break
        if command in ("help", "?"):
            print_help(step)
            continue
        if command == "pos":
            print("Current position: %s" % arm.getPos())
            continue
        if command == "open":
            arm.openGripper()
            continue
        if command == "close":
            arm.closeGripper()
            continue
        if command == "step":
            if len(parts) != 2:
                print("Usage: step N")
                continue
            try:
                step = float(parts[1])
            except ValueError:
                print("Step must be a number.")
                continue
            print("Step is now %.1f mm" % step)
            continue
        if command == "goto":
            if len(parts) != 4:
                print("Usage: goto X Y Z")
                continue
            try:
                target = [float(value) for value in parts[1:]]
            except ValueError:
                print("Coordinates must be numbers.")
                continue
        elif command in ("x+", "x-", "y+", "y-", "z+", "z-"):
            target = [x, y, z]
            axis = "xyz".index(command[0])
            target[axis] += step if command[1] == "+" else -step
        else:
            print("Unknown command. Type 'help' for commands.")
            continue

        if not arm.isReachable(*target):
            print("Unreachable point: %s" % target)
            continue

        arm.gotoPoint(*target)
        print("Current position: %s" % arm.getPos())


if __name__ == "__main__":
    main()
