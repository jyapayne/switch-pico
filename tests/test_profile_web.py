from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Any

import pytest

from switch_pico_bridge import config_manager, profile_web
from tests.test_config_manager import FakeDevice, custom_profile


@contextmanager
def running_server(
    monkeypatch: pytest.MonkeyPatch, device: FakeDevice
) -> Iterator[tuple[str, str]]:
    server = profile_web.ProfileEditorServer(
        ("127.0.0.1", 0),
        bus=None,
        device_address=None,
        timeout=1.0,
    )
    monkeypatch.setattr(server, "find_device", lambda: device)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", server.mutation_token
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def request_json(
    url: str,
    *,
    method: str = "GET",
    value: Any = None,
    token: str | None = None,
) -> tuple[int, dict[str, Any]]:
    data = None
    headers: dict[str, str] = {}
    if value is not None:
        data = json.dumps(value).encode("utf-8")
        headers["Content-Type"] = "application/json"
    if token is not None:
        headers["X-Switch-Pico-Token"] = token
    request = urllib.request.Request(
        url, data=data, headers=headers, method=method
    )
    try:
        with urllib.request.urlopen(request, timeout=2) as response:
            return response.status, json.loads(response.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_editor_serves_assets_and_complete_schema(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    with running_server(monkeypatch, FakeDevice()) as (base_url, _):
        with urllib.request.urlopen(f"{base_url}/", timeout=2) as response:
            page = response.read().decode("utf-8")
            assert response.headers["Content-Security-Policy"]
        with urllib.request.urlopen(f"{base_url}/app.js", timeout=2) as response:
            script_size = len(response.read())
            assert response.headers["Content-Type"].startswith("text/javascript")
        with urllib.request.urlopen(
            f"{base_url}/assets/controller-switch-pro.svg", timeout=2
        ) as response:
            artwork_size = len(response.read())
            assert response.headers["Content-Type"] == "image/svg+xml"


        status, schema = request_json(f"{base_url}/api/schema")

    assert status == 200
    assert "Profile Studio" in page
    assert script_size > 1000
    assert artwork_size > 10000
    assert schema["buttons"] == list(config_manager.LOGICAL_BUTTONS)
    assert schema["controls"] == list(config_manager.LOGICAL_CONTROLS)
    assert schema["rumble_policies"] == list(config_manager.RUMBLE_POLICIES)
    assert schema["turbo_modes"] == list(config_manager.TURBO_MODES)
    assert schema["macro_overrides"] == list(config_manager.MACRO_OVERRIDE_NAMES)
    assert schema["profile_capacity"] == 8
    assert schema["control_labels"]["generic"]["south"] == "A"
    assert schema["control_labels"]["xbox"]["left_shoulder"] == "LB"
    assert schema["control_labels"]["switch"]["east"] == "A"
    assert schema["control_labels"]["switch"]["left_trigger"] == "ZL"
    assert schema["control_labels"]["playstation"]["south"] == "Cross"
    assert schema["control_labels"]["playstation"]["select"] == "Create"
    assert (
        config_manager.ControllerProfile.from_json(
            json.dumps(schema["default_profile"])
        )
        == config_manager.ControllerProfile.default()
    )


def test_editor_identifies_connected_controller_artwork(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    switch = config_manager.ControllerIdentity(
        True,
        config_manager.TRANSPORT_CLASSIC,
        0,
        bytes.fromhex("010203040506"),
        0x057E,
        0x2009,
    )
    dualsense = config_manager.ControllerIdentity(
        True,
        config_manager.TRANSPORT_CLASSIC,
        0,
        bytes.fromhex("111213141516"),
        0x054C,
        0x0CE6,
    )
    device.profile_identities = [
        device.global_identity,
        switch,
        dualsense,
        device.stable_identity,
    ]
    device.active_profiles = {
        identity.to_bytes(): 0 for identity in device.profile_identities
    }

    with running_server(monkeypatch, device) as (base_url, _):
        status, listing = request_json(f"{base_url}/api/profiles")

    assert status == 200
    assert [
        identity["controller"] for identity in listing["identities"]
    ] == [
        {"model": "Generic controller", "style": "generic"},
        {"model": "Nintendo Switch Pro Controller", "style": "switch"},
        {"model": "Sony DualSense", "style": "playstation"},
        {"model": "Xbox controller", "style": "xbox"},
    ]
    assert [identity["label"] for identity in listing["identities"]] == [
        "Default profile",
        "Switch Pro · 05:06",
        "DualSense · 15:16",
        "Xbox · 50:60",
    ]
    assert [identity["key"] for identity in listing["identities"]] == [
        identity.to_bytes().hex() for identity in device.profile_identities
    ]


def test_editor_reads_writes_and_activates_profiles_atomically(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    with running_server(monkeypatch, device) as (base_url, token):
        status, listing = request_json(f"{base_url}/api/profiles")
        assert status == 200
        assert listing["identities"][1]["active_profile"] == 2
        assert listing["identities"][1]["key"] == (
            device.stable_identity.to_bytes().hex()
        )
        assert listing["identities"][1]["controller"] == {
            "model": "Xbox controller",
            "style": "xbox",
        }

        status, playtest = request_json(
            f"{base_url}/api/profiles/1/8/playtest"
        )
        assert status == 200
        assert playtest["connected"] is True
        assert playtest["label"] == "Xbox · 50:60"
        assert playtest["controller"] == {
            "model": "Xbox controller",
            "style": "xbox",
        }
        assert playtest["buttons"] == [
            "south",
            "dpad_up",
            "dpad_right",
        ]
        assert playtest["left_stick"] == {"x": -1234, "y": 2345}
        assert playtest["triggers"] == {"left": 123, "right": 65000}
        assert playtest["battery"] == 100
        assert playtest["capabilities"] == [
            "rumble", "lightbar", "player_leds", "motion"
        ]

        status, selected = request_json(f"{base_url}/api/profiles/1/8")
        assert status == 200
        assert selected["active"] is False

        profile = custom_profile().to_json_object()
        status, stored = request_json(
            f"{base_url}/api/profiles/1/8",
            method="PUT",
            value=profile,
            token=token,
        )
        assert status == 200
        assert stored["stored_generation"] == 8
        assert (
            config_manager.ControllerProfile.from_bytes(
                device.profiles[(device.stable_identity.to_bytes(), 7)]
            )
            == custom_profile()
        )
        assert device.profile_chunk_sizes == [40, 40, 40, 40, 40, 40, 16]
        status, renamed = request_json(
            f"{base_url}/api/profiles/1/8/name",
            method="PUT",
            value={"value": "Desktop"},
            token=token,
        )
        assert status == 200
        assert renamed["stored_generation"] == 9
        assert device.profile_names[
            (device.stable_identity.to_bytes(), 7)
        ] == "Desktop"

        status, aliased = request_json(
            f"{base_url}/api/identities/1/alias",
            method="PUT",
            value={"value": "Desk pad"},
            token=token,
        )
        assert status == 200
        assert aliased["label"] == "Desk pad"

        status, identified = request_json(
            f"{base_url}/api/identities/1/identify",
            method="POST",
            token=token,
        )
        assert status == 200
        assert identified == {"identified": True}
        assert device.identified_identities == [
            device.stable_identity.to_bytes()
        ]

        status, copied = request_json(
            f"{base_url}/api/profiles/1/8/copy",
            method="POST",
            value={"identity_index": 1, "profile_number": 4},
            token=token,
        )
        assert status == 200
        assert copied["stored_generation"] == 12
        assert device.profiles[
            (device.stable_identity.to_bytes(), 3)
        ] == custom_profile().to_bytes()
        assert device.profile_names[
            (device.stable_identity.to_bytes(), 3)
        ] == "Desktop"


        status, activated = request_json(
            f"{base_url}/api/profiles/1/8/activate",
            method="POST",
            token=token,
        )

    assert status == 200
    assert activated["stored_generation"] == 13
    assert device.active_profiles[device.stable_identity.to_bytes()] == 7


def test_editor_rejects_invalid_or_unauthorized_mutations(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    device = FakeDevice()
    original = device.profiles[(device.global_identity.to_bytes(), 0)]
    with running_server(monkeypatch, device) as (base_url, token):
        status, unauthorized = request_json(
            f"{base_url}/api/profiles/0/1",
            method="PUT",
            value=config_manager.ControllerProfile.default().to_json_object(),
        )
        assert status == 403
        assert "token" in unauthorized["error"]

        malformed = config_manager.ControllerProfile.default().to_json_object()
        malformed["sticks"]["left"]["outer_saturation"] = 0
        status, invalid = request_json(
            f"{base_url}/api/profiles/0/1",
            method="PUT",
            value=malformed,
            token=token,
        )

    assert status == 400
    assert "outer_saturation" in invalid["error"]
    assert device.profiles[(device.global_identity.to_bytes(), 0)] == original
