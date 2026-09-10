from pathlib import Path
import shutil
import subprocess


def test_native_packet_fidelity_and_lifecycle(tmp_path: Path) -> None:
    root = Path(__file__).resolve().parents[1]
    cxx = shutil.which("c++") or shutil.which("g++")
    assert cxx is not None, "a host C++ compiler is required"
    probe = root / "tools" / "switch2_usb_probe"
    executable = tmp_path / "switch2_mouse_bridge_test"
    subprocess.run(
        [cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
         "-DSWITCH_PICO_SWITCH2_USB_BRIDGE=1", "-DSWITCH_PICO_BLUEPAD32=1",
         "-DSWITCH_PICO_ENABLE_BLE=1", "-DSWITCH_PICO_SWITCH2_MOUSE_CAPTURE=1",
         "-DSWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=1",
         "-DSWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES=0x98,0xe2,0x55,0x07,0xdf,0x00",
         f"-I{root / 'tests' / 'switch2_mouse_bridge_native_stubs'}",
         f"-I{probe}", f"-I{root / 'src' / 'firmware'}", f"-I{root / 'bluepad32_config'}",
         str(root / "tests" / "switch2_mouse_bridge_test.cpp"),
         str(probe / "controller_input.cpp"),
         str(root / "src" / "firmware" / "input" / "switch2_mouse_capture.cpp"),
         "-o", str(executable)],
        check=True, cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
