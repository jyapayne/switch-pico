# example_switch_macro.py
import time
from switch_pico_uart import SwitchUARTClient, SwitchButton, SwitchDpad, first_serial_port

PORT = first_serial_port(include_descriptions=["USB to UART"]) or "COM5"  # auto-pick first serial port, or fall back
SEND_INTERVAL = 1 / 500  # optional: match controller_uart_bridge default

# Convenience list of every SwitchButton (plus DPAD directions).
ALL_BUTTONS = [
    SwitchButton.A,
    SwitchButton.B,
    SwitchButton.X,
    SwitchButton.Y,
    SwitchButton.L,
    SwitchButton.R,
    SwitchButton.ZL,
    SwitchButton.ZR,
    SwitchButton.PLUS,
    SwitchButton.MINUS,
    SwitchButton.CAPTURE,
    SwitchButton.LCLICK,
    SwitchButton.RCLICK,
    SwitchDpad.DOWN,
    SwitchDpad.UP,
    SwitchDpad.LEFT,
    SwitchDpad.RIGHT,
]


def main() -> None:
    # auto_send keeps the current state flowing in the background, so we don't
    # need to manually pump frames to the Pico.
    with SwitchUARTClient(PORT, send_interval=SEND_INTERVAL, auto_send=True) as client:
        client.neutral()

        # Press every button/DPAD direction one-by-one, holding each briefly.
        for button in ALL_BUTTONS:
            client.press_for(0.10, button)
            time.sleep(0.05)  # short gap between presses

        # Push left stick up briefly.
        client.move_left_stick_for(0.0, -1.0, 0.2)

        # Hold dpad right for one second using hat-friendly press/release.
        client.press_for(0.5, SwitchDpad.RIGHT)

        # Listen for rumble frames for a few seconds while the background sender runs.
        end = time.monotonic() + 3
        while time.monotonic() < end:
            rumble = client.poll_rumble()
            if rumble:
                left, right = rumble
                print(f"Rumble: L={left:.2f} R={right:.2f}")
            time.sleep(0.01)


if __name__ == "__main__":
    main()
