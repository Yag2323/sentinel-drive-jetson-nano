#!/usr/bin/env python3

from __future__ import print_function

import argparse
import atexit
import math
import signal
import sys
import time

import smbus


# ============================================================
# I2C ADDRESSES
# ============================================================

I2C_BUS = 1
PCA9685_ADDRESS = 0x40
INA219_ADDRESS = 0x41


# ============================================================
# PCA9685 REGISTERS
# ============================================================

MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
PRE_SCALE = 0xFE

MODE1_RESTART = 0x80
MODE1_AI = 0x20
MODE1_SLEEP = 0x10

MODE2_OUTDRV = 0x04


# ============================================================
# INA219 REGISTER
# ============================================================

INA219_BUS_VOLTAGE = 0x02


# ============================================================
# PCA9685 TO L298N CHANNEL MAPPING
# ============================================================

LEFT_ENA = 0
LEFT_IN1 = 1
LEFT_IN2 = 2

RIGHT_IN3 = 3
RIGHT_IN4 = 4
RIGHT_ENB = 5


# ============================================================
# SYSTEM LIMITS
# ============================================================

PWM_FREQUENCY_HZ = 200.0

# Conservative limits for a 2S Li-ion battery.
MIN_BATTERY_VOLTAGE = 6.60
MAX_BATTERY_VOLTAGE = 8.60

# Gentle differential steering.
TURN_INNER_WHEEL_FACTOR = 0.35


def swap_word(value):
    """
    Convert SMBus byte order into INA219 register order.
    """
    return (
        ((value & 0x00FF) << 8) |
        ((value & 0xFF00) >> 8)
    )


class ManualMotorController(object):
    """
    Safe manual control for the two L298N motor channels.

    Every movement is a timed pulse. The motors are
    automatically stopped after each command.
    """

    def __init__(
        self,
        reverse_left=False,
        reverse_right=False,
    ):
        self.bus = smbus.SMBus(I2C_BUS)

        self.reverse_left = bool(reverse_left)
        self.reverse_right = bool(reverse_right)

        self.closed = False

        self.configure_pca9685()
        self.all_off()

    def read_register(self, register):
        return self.bus.read_byte_data(
            PCA9685_ADDRESS,
            register,
        )

    def write_register(self, register, value):
        self.bus.write_byte_data(
            PCA9685_ADDRESS,
            register,
            int(value) & 0xFF,
        )

    def configure_pca9685(self):
        """
        Configure PWM frequency and ensure that the
        PCA9685 oscillator is awake.
        """
        old_mode = self.read_register(MODE1)

        prescale_value = (
            25000000.0 /
            (
                4096.0 *
                PWM_FREQUENCY_HZ
            )
        ) - 1.0

        prescale = int(
            math.floor(
                prescale_value + 0.5
            )
        )

        base_mode = old_mode & 0x7F

        sleep_mode = (
            base_mode |
            MODE1_SLEEP
        )

        self.write_register(
            MODE1,
            sleep_mode,
        )

        self.write_register(
            PRE_SCALE,
            prescale,
        )

        awake_mode = (
            (base_mode & ~MODE1_SLEEP) |
            MODE1_AI
        )

        self.write_register(
            MODE1,
            awake_mode,
        )

        time.sleep(0.002)

        self.write_register(
            MODE1,
            awake_mode | MODE1_RESTART,
        )

        time.sleep(0.010)

        self.write_register(
            MODE2,
            MODE2_OUTDRV,
        )

        mode_after = self.read_register(MODE1)

        if mode_after & MODE1_SLEEP:
            raise RuntimeError(
                "PCA9685 remained in SLEEP mode."
            )

        print(
            "PCA9685 configured at {:.1f} Hz.".format(
                PWM_FREQUENCY_HZ
            )
        )

        print(
            "MODE1: 0x{:02X}".format(
                mode_after
            )
        )

    def set_channel_duty(
        self,
        channel,
        duty_fraction,
    ):
        """
        Set one PCA9685 channel from 0.0 to 1.0.
        """
        duty_fraction = max(
            0.0,
            min(
                1.0,
                float(duty_fraction),
            ),
        )

        base_register = (
            LED0_ON_L +
            4 * int(channel)
        )

        if duty_fraction <= 0.0:
            # Full OFF
            values = [
                0x00,
                0x00,
                0x00,
                0x10,
            ]

        elif duty_fraction >= 1.0:
            # Full ON
            values = [
                0x00,
                0x10,
                0x00,
                0x00,
            ]

        else:
            off_count = int(
                round(
                    duty_fraction *
                    4095.0
                )
            )

            values = [
                0x00,
                0x00,
                off_count & 0xFF,
                (off_count >> 8) & 0x0F,
            ]

        for offset, value in enumerate(values):
            self.write_register(
                base_register + offset,
                value,
            )

    def read_battery_voltage(self):
        raw_value = self.bus.read_word_data(
            INA219_ADDRESS,
            INA219_BUS_VOLTAGE,
        )

        voltage_register = swap_word(
            raw_value
        )

        return (
            (voltage_register >> 3) *
            0.004
        )

    def validate_battery(self):
        voltage = self.read_battery_voltage()

        if voltage < MIN_BATTERY_VOLTAGE:
            raise RuntimeError(
                "Motor battery is too low: "
                "{:.3f} V".format(voltage)
            )

        if voltage > MAX_BATTERY_VOLTAGE:
            raise RuntimeError(
                "Unexpectedly high battery voltage: "
                "{:.3f} V".format(voltage)
            )

        return voltage

    def all_off(self):
        """
        Disable both motor speed channels first, then
        remove all direction signals.
        """
        if self.closed:
            return

        self.set_channel_duty(
            LEFT_ENA,
            0.0,
        )

        self.set_channel_duty(
            RIGHT_ENB,
            0.0,
        )

        time.sleep(0.05)

        for channel in (
            LEFT_IN1,
            LEFT_IN2,
            RIGHT_IN3,
            RIGHT_IN4,
        ):
            self.set_channel_duty(
                channel,
                0.0,
            )

    def configure_motor_direction(
        self,
        command,
        input_a,
        input_b,
        reversed_motor,
    ):
        """
        command:
            positive = forward
            negative = backward
            zero     = stop
        """
        effective_command = float(command)

        if reversed_motor:
            effective_command *= -1.0

        if effective_command > 0:
            self.set_channel_duty(
                input_a,
                1.0,
            )

            self.set_channel_duty(
                input_b,
                0.0,
            )

        elif effective_command < 0:
            self.set_channel_duty(
                input_a,
                0.0,
            )

            self.set_channel_duty(
                input_b,
                1.0,
            )

        else:
            self.set_channel_duty(
                input_a,
                0.0,
            )

            self.set_channel_duty(
                input_b,
                0.0,
            )

    def set_motors(
        self,
        left_command,
        right_command,
    ):
        """
        Commands range from -1.0 to +1.0.
        """
        left_command = max(
            -1.0,
            min(
                1.0,
                float(left_command),
            ),
        )

        right_command = max(
            -1.0,
            min(
                1.0,
                float(right_command),
            ),
        )

        # Disable speed before changing direction.
        self.set_channel_duty(
            LEFT_ENA,
            0.0,
        )

        self.set_channel_duty(
            RIGHT_ENB,
            0.0,
        )

        time.sleep(0.08)

        self.configure_motor_direction(
            left_command,
            LEFT_IN1,
            LEFT_IN2,
            self.reverse_left,
        )

        self.configure_motor_direction(
            right_command,
            RIGHT_IN3,
            RIGHT_IN4,
            self.reverse_right,
        )

        time.sleep(0.08)

        self.set_channel_duty(
            LEFT_ENA,
            abs(left_command),
        )

        self.set_channel_duty(
            RIGHT_ENB,
            abs(right_command),
        )

    def run_pulse(
        self,
        label,
        left_command,
        right_command,
        duration,
    ):
        """
        Run one short movement and automatically stop.
        """
        starting_voltage = self.validate_battery()

        print()
        print(
            "{} | Left {:.2f} | Right {:.2f} | "
            "Battery {:.3f} V".format(
                label,
                left_command,
                right_command,
                starting_voltage,
            )
        )

        try:
            self.set_motors(
                left_command,
                right_command,
            )

            start_time = time.time()

            while (
                time.time() -
                start_time <
                duration
            ):
                voltage = self.read_battery_voltage()

                if voltage < MIN_BATTERY_VOLTAGE:
                    raise RuntimeError(
                        "Battery fell below the cutoff: "
                        "{:.3f} V".format(voltage)
                    )

                time.sleep(0.10)

        finally:
            self.all_off()

        ending_voltage = self.read_battery_voltage()

        print(
            "Pulse complete. Motors OFF. "
            "Battery {:.3f} V".format(
                ending_voltage
            )
        )

    def close(self):
        if self.closed:
            return

        try:
            self.all_off()

        finally:
            self.bus.close()
            self.closed = True

            print(
                "Motor controller closed. "
                "All outputs are OFF."
            )


controller = None


def emergency_stop(
    signal_number=None,
    frame=None,
):
    del signal_number
    del frame

    print()
    print("EMERGENCY STOP")

    if controller is not None:
        controller.close()

    raise SystemExit(130)


def print_commands():
    print()
    print("========================================")
    print("MANUAL MOTOR COMMANDS")
    print("========================================")
    print("f  = forward pulse")
    print("b  = backward pulse")
    print("l  = gentle left pulse")
    print("r  = gentle right pulse")
    print("1  = left motor only")
    print("2  = right motor only")
    print("s  = immediate stop")
    print("v  = show motor-battery voltage")
    print("h  = show this help")
    print("q  = stop and quit")
    print()
    print("Every movement stops automatically.")
    print("Ctrl+C also performs an emergency stop.")
    print("========================================")


def main():
    global controller

    parser = argparse.ArgumentParser(
        description=(
            "Safe manual differential-drive control"
        )
    )

    parser.add_argument(
        "--duty",
        type=float,
        default=0.35,
        help=(
            "Motor duty from 0.25 to 0.55. "
            "Default: 0.35"
        ),
    )

    parser.add_argument(
        "--pulse",
        type=float,
        default=0.50,
        help=(
            "Pulse duration from 0.20 to 1.00 seconds. "
            "Default: 0.50"
        ),
    )

    parser.add_argument(
        "--reverse-left",
        action="store_true",
        help="Reverse the left motor software direction.",
    )

    parser.add_argument(
        "--reverse-right",
        action="store_true",
        help="Reverse the right motor software direction.",
    )

    arguments = parser.parse_args()

    if not 0.25 <= arguments.duty <= 0.55:
        print(
            "ERROR: --duty must be from "
            "0.25 to 0.55."
        )
        return 1

    if not 0.20 <= arguments.pulse <= 1.00:
        print(
            "ERROR: --pulse must be from "
            "0.20 to 1.00 seconds."
        )
        return 1

    signal.signal(
        signal.SIGINT,
        emergency_stop,
    )

    signal.signal(
        signal.SIGTERM,
        emergency_stop,
    )

    controller = ManualMotorController(
        reverse_left=arguments.reverse_left,
        reverse_right=arguments.reverse_right,
    )

    atexit.register(
        controller.close
    )

    battery_voltage = (
        controller.validate_battery()
    )

    print()
    print("========================================")
    print("SAFE MANUAL MOTOR CONTROL")
    print("========================================")
    print(
        "Duty: {:.0f}%".format(
            arguments.duty * 100.0
        )
    )
    print(
        "Pulse duration: {:.2f} seconds".format(
            arguments.pulse
        )
    )
    print(
        "Reverse left:",
        arguments.reverse_left,
    )
    print(
        "Reverse right:",
        arguments.reverse_right,
    )
    print(
        "Motor battery: {:.3f} V".format(
            battery_voltage
        )
    )
    print("========================================")

    print_commands()

    duty = arguments.duty
    pulse = arguments.pulse

    try:
        while True:
            command = input(
                "\nEnter command: "
            ).strip().lower()

            if command == "f":
                controller.run_pulse(
                    "FORWARD",
                    duty,
                    duty,
                    pulse,
                )

            elif command == "b":
                controller.run_pulse(
                    "BACKWARD",
                    -duty,
                    -duty,
                    pulse,
                )

            elif command == "l":
                controller.run_pulse(
                    "GENTLE LEFT",
                    (
                        duty *
                        TURN_INNER_WHEEL_FACTOR
                    ),
                    duty,
                    pulse,
                )

            elif command == "r":
                controller.run_pulse(
                    "GENTLE RIGHT",
                    duty,
                    (
                        duty *
                        TURN_INNER_WHEEL_FACTOR
                    ),
                    pulse,
                )

            elif command == "1":
                controller.run_pulse(
                    "LEFT MOTOR ONLY",
                    duty,
                    0.0,
                    pulse,
                )

            elif command == "2":
                controller.run_pulse(
                    "RIGHT MOTOR ONLY",
                    0.0,
                    duty,
                    pulse,
                )

            elif command == "s":
                controller.all_off()
                print("STOPPED. All motor outputs are OFF.")

            elif command == "v":
                voltage = controller.read_battery_voltage()

                print(
                    "Motor battery voltage: "
                    "{:.3f} V".format(voltage)
                )

            elif command == "h":
                print_commands()

            elif command == "q":
                print("Stopping and closing.")
                break

            elif command == "":
                continue

            else:
                print(
                    "Unknown command. Type h for help."
                )

    except KeyboardInterrupt:
        print()
        print("Keyboard interrupt received.")

    finally:
        controller.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
