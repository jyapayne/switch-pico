"""
Python helpers for the switch-pico firmware.

Expose the UART helpers at the package root so callers can import
``switch_pico_bridge`` and access the public API directly.
"""

from .switch_pico_uart import (  # noqa: F401
    SwitchButton,
    SwitchDpad,
    SwitchUARTClient,
    axis_to_stick,
    decode_rumble,
    discover_serial_ports,
    first_serial_port,
    str_to_dpad,
    trigger_to_button,
)

__all__ = [
    "SwitchUARTClient",
    "SwitchButton",
    "SwitchDpad",
    "discover_serial_ports",
    "first_serial_port",
    "axis_to_stick",
    "decode_rumble",
    "str_to_dpad",
    "trigger_to_button",
]
