#!/usr/bin/env python3
"""Temporarily configure ArduCopter for DShot motor beeps and play a tune.

Run with one of: setup, play, restore. Propellers must be removed.
"""

import argparse
import time

from pymavlink import mavutil


DEVICE = "/dev/pixhawk"
BAUD = 115200

# Values observed immediately before this test on 2026-08-06.
ORIGINAL = {
    "MOT_PWM_TYPE": 0,
    "SERVO_DSHOT_ESC": 0,
    "NTF_BUZZ_TYPES": 5,
    "BRD_IO_DSHOT": 0,
}

# DShot150 is the lowest-rate and most cable-noise-tolerant DShot option.
TEST = {
    "MOT_PWM_TYPE": 4,
    "SERVO_DSHOT_ESC": 1,
    # DShot only, so an audible result cannot be mistaken for the built-in buzzer.
    "NTF_BUZZ_TYPES": 2,
    "BRD_IO_DSHOT": 1,
}

# Short, energetic Phrygian-dominant dance phrase inspired by Saint-Saens'
# Bacchanale. DShot ESC beepers quantize pitches, so this is an approximation.
TUNE = "MFT180L8O4D-EF+GAB-C+D>DL16C+D<L8B-AGF+ED-P16"


def connect():
    link = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    heartbeat = link.wait_heartbeat(timeout=12)
    if heartbeat is None:
        raise RuntimeError("No Pixhawk heartbeat received")
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        raise RuntimeError("Pixhawk is armed; refusing to continue")
    print(f"connected system={link.target_system} component={link.target_component} disarmed=true")
    return link


def read_param(link, name):
    link.mav.param_request_read_send(
        link.target_system, link.target_component, name.encode("ascii"), -1
    )
    message = link.recv_match(
        type="PARAM_VALUE",
        condition=f"PARAM_VALUE.param_id == '{name}'",
        blocking=True,
        timeout=4,
    )
    if message is None:
        raise RuntimeError(f"No response for parameter {name}")
    return message.param_value


def set_param(link, name, value):
    link.mav.param_set_send(
        link.target_system,
        link.target_component,
        name.encode("ascii"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if message and message.param_id == name:
            if abs(message.param_value - value) > 0.01:
                raise RuntimeError(
                    f"Pixhawk rejected {name}={value}; reported {message.param_value:g}"
                )
            print(f"set {name}={value}")
            return
    raise RuntimeError(f"No acknowledgement for {name}={value}")


def reboot(link):
    print("rebooting Pixhawk to apply output protocol")
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    link.close()


def configure(values):
    link = connect()
    for name, desired in values.items():
        current = read_param(link, name)
        print(f"current {name}={current:g}")
        if abs(current - desired) > 0.01:
            set_param(link, name, desired)
    reboot(link)


def play():
    link = connect()
    for name, expected in TEST.items():
        actual = read_param(link, name)
        if abs(actual - expected) > 0.01:
            raise RuntimeError(f"{name}={actual:g}, expected {expected}; refusing to play")

    tune = TUNE.encode("ascii")
    first, remainder = tune[:30], tune[30:]
    print("sending short Bacchanale DShot-beeper tune")
    link.mav.play_tune_send(
        link.target_system,
        link.target_component,
        first,
        remainder,
    )
    time.sleep(5)
    link.close()
    print("tune command complete")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("setup", "play", "restore"))
    args = parser.parse_args()
    if args.action == "setup":
        configure(TEST)
    elif args.action == "play":
        play()
    else:
        configure(ORIGINAL)


if __name__ == "__main__":
    main()
