"""Local-only HTTP backend for the graphical controller profile editor."""
from __future__ import annotations

import json
import secrets
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from importlib import resources
from typing import Any, Callable, cast
from urllib.parse import urlsplit

import usb.core

from . import config_manager

_MAXIMUM_REQUEST_BYTES = 64 * 1024
_ASSET_TYPES = {
    "/": ("profile_editor.html", "text/html; charset=utf-8"),
    "/app.js": ("profile_editor.js", "text/javascript; charset=utf-8"),
    "/style.css": ("profile_editor.css", "text/css; charset=utf-8"),
    "/assets/controller-switch-pro.svg": (
        "assets/controller-switch-pro.svg",
        "image/svg+xml",
    ),
    "/assets/controller-dualsense.svg": (
        "assets/controller-dualsense.svg",
        "image/svg+xml",
    ),
    "/assets/controller-xbox.svg": (
        "assets/controller-xbox.svg",
        "image/svg+xml",
    ),
}

_CONTROL_LABELS = {
    "generic": {
        "north": "Y", "east": "B", "south": "A", "west": "X",
        "left_shoulder": "LB", "right_shoulder": "RB",
        "left_trigger": "LT", "right_trigger": "RT",
        "select": "View", "start": "Menu", "capture": "Share",
        "system": "Xbox", "left_stick": "Left Stick",
        "right_stick": "Right Stick",
    },
    "xbox": {
        "north": "Y", "east": "B", "south": "A", "west": "X",
        "left_shoulder": "LB", "right_shoulder": "RB",
        "left_trigger": "LT", "right_trigger": "RT",
        "select": "View", "start": "Menu", "capture": "Share",
        "system": "Xbox", "left_stick": "Left Stick",
        "right_stick": "Right Stick",
    },
    "switch": {
        "north": "X", "east": "A", "south": "B", "west": "Y",
        "left_shoulder": "L", "right_shoulder": "R",
        "left_trigger": "ZL", "right_trigger": "ZR",
        "select": "Minus", "start": "Plus", "capture": "Capture",
        "system": "Home", "left_stick": "Left Stick",
        "right_stick": "Right Stick",
    },
    "playstation": {
        "north": "Triangle", "east": "Circle", "south": "Cross",
        "west": "Square", "left_shoulder": "L1",
        "right_shoulder": "R1", "left_trigger": "L2",
        "right_trigger": "R2", "select": "Create", "start": "Options",
        "capture": "Touchpad", "system": "PS", "left_stick": "L3",
        "right_stick": "R3",
    },
}

def _controller_presentation(
    identity: config_manager.ControllerIdentity,
) -> dict[str, str]:
    if identity.is_global_fallback:
        return {"model": "Generic controller", "style": "generic"}

    known = {
        (0x057E, 0x2009): ("Nintendo Switch Pro Controller", "switch"),
        (0x054C, 0x0CE6): ("Sony DualSense", "playstation"),
        (0x054C, 0x09CC): ("Sony DualShock 4", "playstation"),
    }
    exact = known.get((identity.vendor_id, identity.product_id))
    if exact is not None:
        model, style = exact
        return {"model": model, "style": style}
    if identity.vendor_id == 0x054C:
        return {"model": "Sony controller", "style": "playstation"}
    if identity.vendor_id == 0x045E:
        return {"model": "Xbox controller", "style": "xbox"}
    if identity.vendor_id == 0x2DC8:
        return {"model": "8BitDo controller", "style": "switch"}
    return {"model": "Connected controller", "style": "generic"}



class ProfileEditorServer(HTTPServer):
    """Single-threaded server that serializes USB profile transactions."""

    def __init__(
        self,
        address: tuple[str, int],
        *,
        bus: int | None,
        device_address: int | None,
        timeout: float,
    ) -> None:
        super().__init__(address, ProfileEditorHandler)
        self.bus = bus
        self.device_address = device_address
        self.operation_timeout = timeout
        self.mutation_token = secrets.token_urlsafe(32)

    def find_device(self) -> config_manager.UsbDevice:
        return config_manager.find_pico(
            self.bus, self.device_address, self.operation_timeout
        )


class ProfileEditorHandler(BaseHTTPRequestHandler):
    @property
    def profile_server(self) -> ProfileEditorServer:
        return cast(ProfileEditorServer, self.server)


    def do_GET(self) -> None:
        if not self._local_host_header():
            self.send_error(HTTPStatus.FORBIDDEN)
            return

        path = urlsplit(self.path).path
        if path in _ASSET_TYPES:
            self._serve_asset(*_ASSET_TYPES[path])
            return
        if path == "/api/schema":
            self._send_json(
                {
                    "buttons": list(config_manager.LOGICAL_BUTTONS),
                    "controls": list(config_manager.LOGICAL_CONTROLS),
                    "default_switching_chord": [
                        "left_shoulder",
                        "right_shoulder",
                        "select",
                        "start",
                    ],
                    "default_motion_toggle_chord": [
                        "dpad_up",
                        "right_shoulder",
                        "start",
                    ],
                    "rumble_policies": list(config_manager.RUMBLE_POLICIES),
                    "turbo_modes": list(config_manager.TURBO_MODES),
                    "macro_overrides": list(config_manager.MACRO_OVERRIDE_NAMES),
                    "macro_count": config_manager.PROFILE_MACRO_COUNT,
                    "profile_capacity": config_manager.PROFILE_CAPACITY,
                    "control_labels": _CONTROL_LABELS,
                    "maximum_macro_state_steps": (
                        config_manager.PROFILE_MACRO_STEPS_PER_MACRO
                    ),
                    "maximum_shared_macro_steps": (
                        config_manager.PROFILE_MACRO_STEP_CAPACITY
                    ),
                    "maximum_macro_stream_bytes": (
                        config_manager.PROFILE_MACRO_STREAM_SIZE
                    ),
                    "default_profile": (
                        config_manager.ControllerProfile.default().to_json_object()
                    ),
                    "mutation_token": self.profile_server.mutation_token,
                }
            )
            return
        if path == "/api/profiles":
            self._api_call(self._list_profiles)
            return

        selection = self._parse_profile_path(path)
        if selection is not None:
            identity_index, profile_index, action = selection
            if action is None:
                self._api_call(
                    lambda: self._read_profile(identity_index, profile_index)
                )
                return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_PUT(self) -> None:
        if not self._allow_mutation():
            return
        selection = self._parse_profile_path(urlsplit(self.path).path)
        if selection is None or selection[2] is not None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        identity_index, profile_index, _ = selection
        self._api_call(
            lambda: self._write_profile(identity_index, profile_index)
        )

    def do_POST(self) -> None:
        if not self._allow_mutation():
            return
        selection = self._parse_profile_path(urlsplit(self.path).path)
        if selection is None or selection[2] != "activate":
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        identity_index, profile_index, _ = selection
        self._api_call(
            lambda: self._activate_profile(identity_index, profile_index)
        )

    def _local_host_header(self) -> bool:
        host = self.headers.get("Host", "")
        return host in {
            f"127.0.0.1:{self.profile_server.server_port}",
            f"localhost:{self.profile_server.server_port}",
        }

    def _allow_mutation(self) -> bool:
        if not self._local_host_header():
            self.send_error(HTTPStatus.FORBIDDEN)
            return False
        if (
            self.headers.get("X-Switch-Pico-Token")
            != self.profile_server.mutation_token
        ):
            self._send_json(
                {"error": "missing or invalid mutation token"},
                status=HTTPStatus.FORBIDDEN,
            )
            return False
        return True

    @staticmethod
    def _parse_profile_path(path: str) -> tuple[int, int, str | None] | None:
        parts = path.strip("/").split("/")
        if len(parts) not in (4, 5) or parts[:2] != ["api", "profiles"]:
            return None
        try:
            identity_index = int(parts[2])
            profile_number = int(parts[3])
        except ValueError:
            return None
        if (
            identity_index < 0
            or not 1 <= profile_number <= config_manager.PROFILE_CAPACITY
        ):
            return None
        action = parts[4] if len(parts) == 5 else None
        return identity_index, profile_number - 1, action

    def _entries_and_identity(
        self, device: config_manager.UsbDevice, identity_index: int
    ) -> tuple[
        tuple[config_manager.ProfileListEntry, ...],
        config_manager.ControllerIdentity,
    ]:
        entries = config_manager.list_profiles(device)
        if not 0 <= identity_index < len(entries):
            raise config_manager.ConfigManagerError(
                f"identity index {identity_index} is out of range; "
                f"refresh the profile library"
            )
        return entries, entries[identity_index].identity

    def _list_profiles(self) -> dict[str, Any]:
        device = self.profile_server.find_device()
        entries = config_manager.list_profiles(device)
        return {
            "identities": [
                {
                    "index": index,
                    "label": (
                        "Global fallback"
                        if entry.identity.is_global_fallback
                        else (
                            f"{entry.identity.transport_text} "
                            f"{entry.identity.address_text} · "
                            f"{entry.identity.vendor_id:04X}:"
                            f"{entry.identity.product_id:04X}"
                        )
                    ),
                    "active_profile": entry.active_profile_index + 1,
                    "controller": _controller_presentation(entry.identity),
                }
                for index, entry in enumerate(entries)
            ]
        }

    def _read_profile(
        self, identity_index: int, profile_index: int
    ) -> dict[str, Any]:
        device = self.profile_server.find_device()
        entries, identity = self._entries_and_identity(device, identity_index)
        profile = config_manager.read_profile(device, identity, profile_index)
        return {
            "profile": profile.to_json_object(),
            "active": entries[identity_index].active_profile_index == profile_index,
        }

    def _read_json_body(self) -> str:
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError as exc:
            raise config_manager.ConfigManagerError(
                "invalid request content length"
            ) from exc
        if not 1 <= length <= _MAXIMUM_REQUEST_BYTES:
            raise config_manager.ConfigManagerError(
                "profile request must contain 1 to 65536 bytes"
            )
        return self.rfile.read(length).decode("utf-8")

    def _write_profile(
        self, identity_index: int, profile_index: int
    ) -> dict[str, Any]:
        profile = config_manager.ControllerProfile.from_json(self._read_json_body())
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        status = config_manager.write_profile(
            device,
            identity,
            profile_index,
            profile,
            self.profile_server.operation_timeout,
        )
        return {
            "stored_generation": status.stored_generation,
            "stored_crc": f"{status.stored_crc:08x}",
        }

    def _activate_profile(
        self, identity_index: int, profile_index: int
    ) -> dict[str, Any]:
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        status = config_manager.activate_profile(
            device,
            identity,
            profile_index,
            self.profile_server.operation_timeout,
        )
        return {"stored_generation": status.stored_generation}

    def _api_call(
        self, operation: Callable[[], dict[str, Any]]
    ) -> None:
        try:
            result = operation()
        except config_manager.ConfigManagerError as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except usb.core.USBError as exc:
            self._send_json(
                {"error": f"USB access failed: {exc}"},
                status=HTTPStatus.SERVICE_UNAVAILABLE,
            )
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self._send_json(
                {"error": f"invalid JSON request: {exc}"},
                status=HTTPStatus.BAD_REQUEST,
            )
        else:
            self._send_json(result)

    def _serve_asset(self, filename: str, content_type: str) -> None:
        try:
            payload = (
                resources.files("switch_pico_bridge")
                .joinpath("web", filename)
                .read_bytes()
            )
        except (FileNotFoundError, OSError):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self._send_bytes(payload, content_type)

    def _send_json(
        self, value: Any, *, status: HTTPStatus = HTTPStatus.OK
    ) -> None:
        self._send_bytes(
            json.dumps(value, separators=(",", ":")).encode("utf-8"),
            "application/json; charset=utf-8",
            status=status,
        )

    def _send_bytes(
        self,
        payload: bytes,
        content_type: str,
        *,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self'; img-src 'self'; frame-ancestors 'none'",
        )
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format: str, *args: Any) -> None:
        return


def run_profile_editor(
    *,
    bus: int | None,
    address: int | None,
    timeout: float,
    port: int,
    open_browser: bool,
) -> None:
    try:
        server = ProfileEditorServer(
            ("127.0.0.1", port),
            bus=bus,
            device_address=address,
            timeout=timeout,
        )
    except OSError as exc:
        raise config_manager.ConfigManagerError(
            f"could not start profile editor on port {port}: {exc}"
        ) from exc

    url = f"http://127.0.0.1:{server.server_port}/"
    print(f"Profile editor: {url}")
    print("Press Ctrl+C to stop.")
    if open_browser and not webbrowser.open(url):
        print(f"Could not open a browser automatically; open {url} manually.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
