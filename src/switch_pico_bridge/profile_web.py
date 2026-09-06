"""Local-only HTTP backend for the graphical controller profile editor."""

from __future__ import annotations

import json
import secrets
import webbrowser
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer
from importlib import resources
from typing import Any, Callable, cast
from urllib.parse import parse_qs, urlsplit

import usb.core

from . import config_manager

_MAXIMUM_REQUEST_BYTES = 64 * 1024
_ASSET_TYPES = {
    "/": ("profile_editor.html", "text/html; charset=utf-8"),
    "/app.js": ("profile_editor.js", "text/javascript; charset=utf-8"),
    "/playtest.js": (
        "profile_playtest.js",
        "text/javascript; charset=utf-8",
    ),
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
        "north": "Y",
        "east": "B",
        "south": "A",
        "west": "X",
        "left_shoulder": "LB",
        "right_shoulder": "RB",
        "left_trigger": "LT",
        "right_trigger": "RT",
        "select": "View",
        "start": "Menu",
        "capture": "Share",
        "system": "Xbox",
        "left_stick": "Left Stick",
        "right_stick": "Right Stick",
    },
    "xbox": {
        "north": "Y",
        "east": "B",
        "south": "A",
        "west": "X",
        "left_shoulder": "LB",
        "right_shoulder": "RB",
        "left_trigger": "LT",
        "right_trigger": "RT",
        "select": "View",
        "start": "Menu",
        "capture": "Share",
        "system": "Xbox",
        "left_stick": "Left Stick",
        "right_stick": "Right Stick",
    },
    "switch": {
        "north": "X",
        "east": "A",
        "south": "B",
        "west": "Y",
        "left_shoulder": "L",
        "right_shoulder": "R",
        "left_trigger": "ZL",
        "right_trigger": "ZR",
        "select": "Minus",
        "start": "Plus",
        "capture": "Capture",
        "system": "Home",
        "left_stick": "Left Stick",
        "right_stick": "Right Stick",
    },
    "playstation": {
        "north": "Triangle",
        "east": "Circle",
        "south": "Cross",
        "west": "Square",
        "left_shoulder": "L1",
        "right_shoulder": "R1",
        "left_trigger": "L2",
        "right_trigger": "R2",
        "select": "Create",
        "start": "Options",
        "capture": "Touchpad",
        "system": "PS",
        "left_stick": "L3",
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


def _controller_label(
    identity: config_manager.ControllerIdentity, alias: str = ""
) -> str:
    if alias:
        return alias
    if identity.is_global_fallback:
        return "Default profile"
    presentation = _controller_presentation(identity)
    friendly_models = {
        "Nintendo Switch Pro Controller": "Switch Pro",
        "Sony DualSense": "DualSense",
        "Sony DualShock 4": "DualShock 4",
        "Sony controller": "PlayStation Controller",
        "Xbox controller": "Xbox",
        "8BitDo controller": "8BitDo",
        "Connected controller": "Controller",
    }
    model = friendly_models.get(presentation["model"], presentation["model"])
    address_suffix = ":".join(f"{octet:02X}" for octet in identity.address[-2:])
    return f"{model} · {address_suffix}"


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
        self._device: config_manager.UsbDevice | None = None
        self.capture: dict[str, Any] | None = None

    def find_device(self) -> config_manager.UsbDevice:
        if self._device is None:
            self._device = config_manager.find_pico(
                self.bus, self.device_address, self.operation_timeout
            )
        return self._device

    def invalidate_device(self) -> None:
        self._device = None


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
                    "shift_modes": list(config_manager.SHIFT_MODES),
                    "shortcut_selectors": list(
                        config_manager.SHORTCUT_SELECTOR_BUTTONS
                    ),
                    "turbo_settings_bounds": {
                        "rate_hz": {
                            "min": config_manager.PROFILE_TURBO_RATE_MIN,
                            "max": config_manager.PROFILE_TURBO_RATE_MAX,
                        },
                        "duty_percent": {
                            "min": config_manager.PROFILE_TURBO_DUTY_MIN,
                            "max": config_manager.PROFILE_TURBO_DUTY_MAX,
                        },
                        "burst_count": {
                            "min": config_manager.PROFILE_TURBO_BURST_MIN,
                            "max": config_manager.PROFILE_TURBO_BURST_MAX,
                        },
                    },
                    "macro_playback_modes": list(config_manager.MACRO_PLAYBACK_MODES),
                    "macro_repeat_bounds": {
                        "min": config_manager.PROFILE_MACRO_REPEAT_MIN,
                        "max": config_manager.PROFILE_MACRO_REPEAT_MAX,
                    },
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
        capture = self._parse_capture_path(path)
        if capture is not None:
            identity_index, profile_index, action = capture
            if action.isdecimal() or action == "current":
                self._api_call(
                    lambda: self._read_capture(identity_index, profile_index, action)
                )
                return

        selection = self._parse_profile_path(path)
        if selection is not None:
            identity_index, profile_index, action = selection
            if action is None:
                self._api_call(
                    lambda: self._read_profile(identity_index, profile_index)
                )
                return
            if action == "playtest":
                self._api_call(
                    lambda: self._read_playtest(identity_index, profile_index)
                )
                return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_PUT(self) -> None:
        if not self._allow_mutation():
            return
        path = urlsplit(self.path).path
        identity_selection = self._parse_identity_path(path)
        if identity_selection is not None:
            identity_index, action = identity_selection
            if action == "alias":
                self._api_call(lambda: self._set_alias(identity_index))
                return
        selection = self._parse_profile_path(path)
        if selection is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        identity_index, profile_index, action = selection
        if action is None:
            self._api_call(lambda: self._write_profile(identity_index, profile_index))
            return
        if action == "name":
            self._api_call(
                lambda: self._set_profile_name(identity_index, profile_index)
            )
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if not self._allow_mutation():
            return
        path = urlsplit(self.path).path
        if path == "/api/profiles/validate":
            self._api_call(
                lambda: {
                    "profile": config_manager.ControllerProfile.from_json(
                        self._read_json_body()
                    ).to_json_object()
                }
            )
            return
        capture = self._parse_capture_path(path)
        if capture is not None:
            identity_index, profile_index, action = capture
            if action == "start":
                self._api_call(
                    lambda: self._start_capture(identity_index, profile_index)
                )
                return
            if action == "stop":
                self._api_call(
                    lambda: self._stop_capture(identity_index, profile_index)
                )
                return
        identity_selection = self._parse_identity_path(path)
        if identity_selection is not None:
            identity_index, action = identity_selection
            if action == "identify":
                self._api_call(lambda: self._identify(identity_index))
                return
        selection = self._parse_profile_path(path)
        if selection is not None:
            identity_index, profile_index, action = selection
            if action == "activate":
                self._api_call(
                    lambda: self._activate_profile(identity_index, profile_index)
                )
                return
            if action == "copy":
                self._api_call(
                    lambda: self._copy_profile(identity_index, profile_index)
                )
                return
        self.send_error(HTTPStatus.NOT_FOUND)

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

    @classmethod
    def _parse_capture_path(cls, path: str) -> tuple[int, int, str] | None:
        parts = path.strip("/").split("/")
        if len(parts) != 6 or parts[4] != "capture":
            return None
        selection = cls._parse_profile_path("/".join(parts[:4]))
        if selection is None:
            return None
        return selection[0], selection[1], parts[5]

    @staticmethod
    def _parse_identity_path(path: str) -> tuple[int, str] | None:
        parts = path.strip("/").split("/")
        if len(parts) != 4 or parts[:2] != ["api", "identities"]:
            return None
        try:
            identity_index = int(parts[2])
        except ValueError:
            return None
        return (identity_index, parts[3]) if identity_index >= 0 else None

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
                    "label": _controller_label(entry.identity, entry.alias),
                    "key": entry.identity.to_bytes().hex(),
                    "active_profile": entry.active_profile_index + 1,
                    "controller": _controller_presentation(entry.identity),
                    "alias": entry.alias,
                    "modifier_controls": list(
                        config_manager.LOGICAL_CONTROLS
                        if _controller_presentation(entry.identity)["style"]
                        in {"xbox", "playstation"}
                        else config_manager.LOGICAL_BUTTONS
                    ),
                }
                for index, entry in enumerate(entries)
            ]
        }

    def _read_profile(self, identity_index: int, profile_index: int) -> dict[str, Any]:
        device = self.profile_server.find_device()
        entries, identity = self._entries_and_identity(device, identity_index)
        profile = config_manager.read_profile(device, identity, profile_index)
        metadata = config_manager.read_selected_profile_metadata(device)
        return {
            "profile": profile.to_json_object(),
            "name": metadata.profile_names[profile_index],
            "profile_names": list(metadata.profile_names),
            "alias": metadata.alias,
            "active": entries[identity_index].active_profile_index == profile_index,
        }

    def _read_playtest(self, identity_index: int, profile_index: int) -> dict[str, Any]:
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        config_manager.select_profile(device, identity, profile_index)
        playtest = config_manager.read_profile_playtest(device)
        result = playtest.to_json_object()
        result["owner_key"] = identity.to_bytes().hex()
        result["identity_key"] = (
            playtest.identity.to_bytes().hex()
            if playtest.identity is not None
            else None
        )
        result["controller"] = (
            _controller_presentation(playtest.identity)
            if playtest.identity is not None
            else None
        )
        result["label"] = (
            _controller_label(playtest.identity)
            if playtest.identity is not None
            else None
        )
        return result

    @staticmethod
    def _capture_integer(
        body: dict[str, Any], name: str, minimum: int, maximum: int
    ) -> int:
        value = body.get(name)
        if type(value) is not int or not minimum <= value <= maximum:
            raise config_manager.ConfigManagerError(
                f"{name} must be an integer from {minimum} to {maximum}"
            )
        return value

    def _start_capture(self, identity_index: int, profile_index: int) -> dict[str, Any]:
        body = self._read_json_object()
        channels = self._capture_integer(body, "channels", 1, 31)
        maximum = self._capture_integer(body, "max_events", 1, 8)
        axis_quantum = self._capture_integer(body, "axis_quantum", 1, 32767)
        trigger_quantum = self._capture_integer(body, "trigger_quantum", 1, 65535)
        duration = self._capture_integer(body, "max_duration_ms", 1, 80000)
        slot = self._capture_integer(
            body, "slot", 0, config_manager.PROFILE_PLAYTEST_SLOT_COUNT - 1
        )
        generation = self._capture_integer(body, "connection_generation", 0, 0xFFFFFFFF)
        macro_index = self._capture_integer(
            body, "macro_index", 0, config_manager.PROFILE_MACRO_COUNT - 1
        )
        owner_key = body.get("owner_key")
        if not isinstance(owner_key, str):
            raise config_manager.ConfigManagerError("capture owner key is required")
        capture_id = body.get("capture_id")
        if not isinstance(capture_id, str) or not 1 <= len(capture_id) <= 128:
            raise config_manager.ConfigManagerError("capture request ID is required")
        profile = config_manager.ControllerProfile.from_json_object(body.get("profile"))
        others = [
            step
            for index, macro in enumerate(profile.macros)
            if index != macro_index
            for step in macro.steps
        ]
        step_bytes = 3 + sum(
            size for bit, size in enumerate((2, 4, 4, 2, 2)) if channels & (1 << bit)
        )
        available = min(
            config_manager.PROFILE_MACRO_STEPS_PER_MACRO,
            config_manager.PROFILE_MACRO_STEP_CAPACITY - len(others),
            (
                config_manager.PROFILE_MACRO_STREAM_SIZE
                - sum(len(step.to_sparse_bytes()) for step in others)
            )
            // step_bytes,
        )
        if maximum > available:
            raise config_manager.ConfigManagerError(
                "recording would exceed the draft's 8/16/136 macro budget"
            )
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        if identity.to_bytes().hex() != owner_key:
            raise config_manager.ConfigManagerError(
                "profile owner changed; refresh before recording"
            )
        # The routed playtest endpoint selected this owner. Recheck its live
        # identity and generation without changing USB selection on a stale start.
        sample = config_manager.read_profile_playtest(device)
        if (
            not sample.connected
            or sample.identity is None
            or (not identity.is_global_fallback and sample.identity != identity)
            or sample.slot_index != slot
            or sample.connection_generation != generation
        ):
            raise config_manager.ConfigManagerError(
                "controller connection changed; wait for live input before recording"
            )
        previous = config_manager.read_macro_capture(device)
        if previous.state_name == "recording":
            raise config_manager.ConfigManagerError(
                "a recording is already running; stop it before starting another"
            )
        self.profile_server.capture = {
            "identity_index": identity_index,
            "profile_index": profile_index,
            "owner_key": owner_key,
            "macro_index": macro_index,
            "run_id": None,
            "previous_run_id": previous.run_id,
            "slot": slot,
            "connection_generation": generation,
            "result": None,
            "capture_id": capture_id,
            "channels": channels,
            "max_events": maximum,
            "axis_quantum": axis_quantum,
            "trigger_quantum": trigger_quantum,
            "max_duration_ms": duration,
        }
        page = config_manager.start_macro_capture(
            device,
            slot,
            generation,
            channels=channels,
            max_events=maximum,
            axis_quantum=axis_quantum,
            trigger_quantum=trigger_quantum,
            max_duration_ms=duration,
        )
        return self._capture_result(device, page)

    def _bound_capture(
        self,
        identity_index: int,
        profile_index: int,
        run_id: int | None,
        owner_key: str | None,
        generation: int,
        capture_id: str | None,
    ) -> dict[str, Any]:
        capture = self.profile_server.capture
        if (
            capture is None
            or capture["identity_index"] != identity_index
            or capture["profile_index"] != profile_index
            or capture["owner_key"] != owner_key
            or capture["capture_id"] != capture_id
            or capture["connection_generation"] != generation
            or (run_id is not None and capture["run_id"] != run_id)
        ):
            raise config_manager.ConfigManagerError(
                "stale capture run, connection or profile owner"
            )
        return capture

    def _capture_device(self, capture: dict[str, Any]) -> config_manager.UsbDevice:
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, capture["identity_index"])
        if identity.to_bytes().hex() != capture["owner_key"]:
            raise config_manager.ConfigManagerError(
                "capture profile owner moved; refusing a stale library index"
            )
        return device

    def _capture_result(
        self,
        device: config_manager.UsbDevice,
        page: config_manager.MacroCapturePage,
    ) -> dict[str, Any]:
        capture = self.profile_server.capture
        assert capture is not None
        run_id = capture["run_id"]
        if (
            not page.run_id
            or page.run_id == capture["previous_run_id"]
            or (run_id is not None and page.run_id != run_id)
            or any(
                getattr(page, field) != capture[field]
                for field in (
                    "slot",
                    "connection_generation",
                    "channels",
                    "max_events",
                    "axis_quantum",
                    "trigger_quantum",
                    "max_duration_ms",
                )
            )
        ):
            raise config_manager.ConfigManagerError(
                "capture run, connection or options changed"
            )
        capture["run_id"] = page.run_id
        terminal = page.state_name not in {"idle", "recording"}
        if terminal:
            page = config_manager.collect_macro_capture(device, page.run_id)
            if any(
                getattr(page, field) != capture[field]
                for field in (
                    "run_id",
                    "slot",
                    "connection_generation",
                    "channels",
                    "max_events",
                    "axis_quantum",
                    "trigger_quantum",
                    "max_duration_ms",
                )
            ):
                raise config_manager.ConfigManagerError(
                    "capture changed during collection"
                )
        result = page.to_json_object()
        result.update(
            {
                "owner_key": capture["owner_key"],
                "capture_id": capture["capture_id"],
                "profile_number": capture["profile_index"] + 1,
                "macro_index": capture["macro_index"],
            }
        )
        if terminal:
            try:
                result["steps"] = [
                    step.to_json_object()
                    for step in config_manager.capture_macro_steps(page)
                ]
            except config_manager.ConfigManagerError as exc:
                # A conversion failure must not hide the retained raw events.
                result["conversion_error"] = str(exc)
            capture["result"] = result
        return result

    def _read_capture(
        self, identity_index: int, profile_index: int, action: str
    ) -> dict[str, Any]:
        query = parse_qs(urlsplit(self.path).query)
        try:
            generation = int(query.get("connection_generation", [""])[0])
        except ValueError as exc:
            raise config_manager.ConfigManagerError(
                "capture connection generation is required"
            ) from exc
        capture = self._bound_capture(
            identity_index,
            profile_index,
            None if action == "current" else int(action),
            query.get("owner_key", [None])[0],
            generation,
            query.get("capture_id", [None])[0],
        )
        device = self._capture_device(capture)
        if capture["result"] is not None:
            return cast(dict[str, Any], capture["result"])
        page = config_manager.read_macro_capture(device, capture["run_id"] or 0)
        return self._capture_result(device, page)

    def _stop_capture(self, identity_index: int, profile_index: int) -> dict[str, Any]:
        body = self._read_json_object()
        run_id = self._capture_integer(body, "run_id", 1, 0xFFFFFFFF)
        generation = self._capture_integer(body, "connection_generation", 0, 0xFFFFFFFF)
        capture = self._bound_capture(
            identity_index,
            profile_index,
            run_id,
            body.get("owner_key"),
            generation,
            body.get("capture_id"),
        )
        device = self._capture_device(capture)
        if capture["result"] is not None:
            return cast(dict[str, Any], capture["result"])
        page = config_manager.read_macro_capture(device, run_id)
        if (
            page.slot != capture["slot"]
            or page.connection_generation != generation
            or page.run_id != run_id
        ):
            raise config_manager.ConfigManagerError("capture connection or run changed")
        if page.state_name == "recording":
            page = config_manager.stop_macro_capture(device, run_id)
        return self._capture_result(device, page)

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

    def _read_json_object(self) -> dict[str, Any]:
        value = json.loads(self._read_json_body())
        if not isinstance(value, dict):
            raise config_manager.ConfigManagerError(
                "request body must be a JSON object"
            )
        return value

    def _write_profile(self, identity_index: int, profile_index: int) -> dict[str, Any]:
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

    def _set_profile_name(
        self, identity_index: int, profile_index: int
    ) -> dict[str, Any]:
        value = self._read_json_object().get("value")
        if not isinstance(value, str):
            raise config_manager.ConfigManagerError("profile name must be text")
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        status = config_manager.set_profile_metadata(
            device,
            identity,
            profile_index,
            value,
            self.profile_server.operation_timeout,
        )
        return {"stored_generation": status.stored_generation}

    def _set_alias(self, identity_index: int) -> dict[str, Any]:
        value = self._read_json_object().get("value")
        if not isinstance(value, str):
            raise config_manager.ConfigManagerError("controller alias must be text")
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        status = config_manager.set_profile_metadata(
            device,
            identity,
            config_manager.PROFILE_NONE_BUTTON,
            value,
            self.profile_server.operation_timeout,
        )
        return {
            "stored_generation": status.stored_generation,
            "label": _controller_label(identity, value),
        }

    def _identify(self, identity_index: int) -> dict[str, Any]:
        device = self.profile_server.find_device()
        _, identity = self._entries_and_identity(device, identity_index)
        config_manager.identify_controller(device, identity)
        return {"identified": True}

    def _copy_profile(self, identity_index: int, profile_index: int) -> dict[str, Any]:
        destination = self._read_json_object()
        destination_identity_index = destination.get("identity_index")
        destination_profile_number = destination.get("profile_number")
        if (
            type(destination_identity_index) is not int
            or type(destination_profile_number) is not int
            or not 1 <= destination_profile_number <= config_manager.PROFILE_CAPACITY
        ):
            raise config_manager.ConfigManagerError("copy destination is invalid")
        profile = config_manager.ControllerProfile.from_json_object(
            destination.get("profile")
        )
        name = destination.get("name")
        if (
            type(name) is not str
            or "\x00" in name
            or len(name.encode("utf-8")) > config_manager.PROFILE_METADATA_MAX_BYTES
        ):
            raise config_manager.ConfigManagerError(
                "profile name must contain at most 31 UTF-8 bytes"
            )
        device = self.profile_server.find_device()
        self._entries_and_identity(device, identity_index)
        _, destination_identity = self._entries_and_identity(
            device, destination_identity_index
        )
        target_profile = destination_profile_number - 1
        profile_status = config_manager.write_profile(
            device,
            destination_identity,
            target_profile,
            profile,
            self.profile_server.operation_timeout,
        )
        name_status = config_manager.set_profile_metadata(
            device,
            destination_identity,
            target_profile,
            name,
            self.profile_server.operation_timeout,
        )
        return {
            "stored_generation": name_status.stored_generation,
            "profile_generation": profile_status.stored_generation,
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

    def _api_call(self, operation: Callable[[], dict[str, Any]]) -> None:
        try:
            result = operation()
        except config_manager.ConfigManagerError as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except usb.core.USBError as exc:
            self.profile_server.invalidate_device()
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

    def _send_json(self, value: Any, *, status: HTTPStatus = HTTPStatus.OK) -> None:
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
