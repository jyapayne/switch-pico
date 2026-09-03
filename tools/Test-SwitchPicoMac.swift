#!/usr/bin/env swift

import Darwin
import Dispatch
import Foundation
import IOKit.hid

private let vendorID = 0xCAFE
private let productID = 0x4021
private let genericDesktopPage = 0x01
private let simulationControlsPage = 0x02
private let gamePadUsage = 0x05
private let buttonPage = 0x09
private let expectedInterfaceCount = 4

private func writeError(_ message: String) {
    FileHandle.standardError.write(Data((message + "\n").utf8))
}

private func ioReturnDescription(_ result: IOReturn) -> String {
    String(format: "0x%08X", UInt32(bitPattern: Int32(result)))
}

private func printInputMonitoringHint() {
    writeError(
        "If macOS denied HID access, enable your terminal (or the app launching Swift) in "
            + "System Settings > Privacy & Security > Input Monitoring, then rerun the command."
    )
}

private func numberProperty(_ device: IOHIDDevice, keys: [String]) -> Int? {
    for key in keys {
        if let number = IOHIDDeviceGetProperty(device, key as CFString) as? NSNumber {
            return number.intValue
        }
    }
    return nil
}

private func deviceKey(_ device: IOHIDDevice) -> ObjectIdentifier {
    ObjectIdentifier(device)
}

private struct DeviceIdentity {
    let interface: String
    let location: String

    init(_ device: IOHIDDevice) {
        if let value = numberProperty(device, keys: ["InterfaceID", "InterfaceNumber", "bInterfaceNumber"]) {
            interface = String(value)
        } else {
            interface = "unknown"
        }

        if let value = numberProperty(device, keys: ["LocationID", "locationID"]) {
            location = String(format: "0x%08X", UInt32(truncatingIfNeeded: value))
        } else {
            location = "unknown"
        }
    }

    var prefix: String {
        "[interface=\(interface) location=\(location)]"
    }
}

private final class DeviceState {
    let device: IOHIDDevice
    let identity: DeviceIdentity
    var lastValues: [Int: CFIndex] = [:]

    init(device: IOHIDDevice) {
        self.device = device
        identity = DeviceIdentity(device)
    }
}

private final class HIDMonitor {
    private let manager: IOHIDManager
    private let runLoop: CFRunLoop
    private var devices: [ObjectIdentifier: DeviceState] = [:]
    private var interruptSource: DispatchSourceSignal?
    private var didPrintPermissionHint = false
    private var isCleanedUp = false

    init(manager: IOHIDManager, runLoop: CFRunLoop) {
        self.manager = manager
        self.runLoop = runLoop
    }

    func start() -> IOReturn {
        let context = Unmanaged.passUnretained(self).toOpaque()
        let matching: [String: Any] = [
            kIOHIDVendorIDKey as String: vendorID,
            kIOHIDProductIDKey as String: productID,
            kIOHIDDeviceUsagePageKey as String: genericDesktopPage,
            kIOHIDDeviceUsageKey as String: gamePadUsage,
        ]

        IOHIDManagerSetDeviceMatching(manager, matching as CFDictionary)
        IOHIDManagerRegisterDeviceMatchingCallback(manager, deviceMatchedCallback, context)
        IOHIDManagerRegisterDeviceRemovalCallback(manager, deviceRemovedCallback, context)
        IOHIDManagerRegisterInputValueCallback(manager, inputValueCallback, context)
        IOHIDManagerScheduleWithRunLoop(manager, runLoop, CFRunLoopMode.defaultMode.rawValue)

        let result = IOHIDManagerOpen(manager, IOOptionBits(kIOHIDOptionsTypeNone))
        guard result == kIOReturnSuccess else {
            writeError("Could not open the IOHID manager (\(ioReturnDescription(result))).")
            printPermissionHintOnce()
            return result
        }

        let matchedDeviceCount: Int
        if let matchedDevices = IOHIDManagerCopyDevices(manager) {
            matchedDeviceCount = CFSetGetCount(matchedDevices)
        } else {
            matchedDeviceCount = 0
        }
        if matchedDeviceCount == 0 {
            print(
                "No CAFE:4021 Generic Desktop Game Pad interfaces are connected; "
                    + "waiting for hot-plug. Press Ctrl-C to stop."
            )
        } else {
            print(
                "Monitoring \(matchedDeviceCount)/\(expectedInterfaceCount) CAFE:4021 "
                    + "raw Game Pad interfaces. Press Ctrl-C to stop."
            )
        }

        signal(SIGINT, SIG_IGN)
        let source = DispatchSource.makeSignalSource(signal: SIGINT, queue: .global(qos: .userInitiated))
        source.setEventHandler { [weak self] in
            guard let self else { return }
            CFRunLoopStop(self.runLoop)
        }
        source.resume()
        interruptSource = source
        return result
    }

    func deviceMatched(_ device: IOHIDDevice, result: IOReturn) {
        guard result == kIOReturnSuccess else {
            writeError("Device matching callback failed (\(ioReturnDescription(result))).")
            return
        }

        let key = deviceKey(device)
        guard devices[key] == nil else { return }

        let identity = DeviceIdentity(device)
        let openResult = IOHIDDeviceOpen(device, IOOptionBits(kIOHIDOptionsTypeNone))
        guard openResult == kIOReturnSuccess else {
            writeError("\(identity.prefix) Could not open HID interface (\(ioReturnDescription(openResult))).")
            printPermissionHintOnce()
            return
        }

        devices[key] = DeviceState(device: device)
        print("\(identity.prefix) attached (\(devices.count)/\(expectedInterfaceCount) Game Pad interfaces open)")
    }

    func deviceRemoved(_ device: IOHIDDevice, result: IOReturn) {
        let key = deviceKey(device)
        let identity = devices.removeValue(forKey: key)?.identity ?? DeviceIdentity(device)
        IOHIDDeviceClose(device, IOOptionBits(kIOHIDOptionsTypeNone))

        if result == kIOReturnSuccess {
            print("\(identity.prefix) removed (\(devices.count)/\(expectedInterfaceCount) Game Pad interfaces open)")
        } else {
            writeError("\(identity.prefix) removal callback failed (\(ioReturnDescription(result))).")
        }
    }

    func inputValue(_ value: IOHIDValue, result: IOReturn) {
        guard result == kIOReturnSuccess else {
            writeError("Input callback failed (\(ioReturnDescription(result))).")
            return
        }

        let element = IOHIDValueGetElement(value)
        let usagePage = IOHIDElementGetUsagePage(element)
        let usage = IOHIDElementGetUsage(element)
        guard let name = usageName(page: usagePage, usage: usage) else { return }

        let device = IOHIDElementGetDevice(element)
        let key = deviceKey(device)
        guard let state = devices[key] else { return }

        let integerValue = IOHIDValueGetIntegerValue(value)
        let elementCookie = Int(IOHIDElementGetCookie(element))
        guard state.lastValues[elementCookie] != integerValue else { return }
        state.lastValues[elementCookie] = integerValue

        let logicalMin = IOHIDElementGetLogicalMin(element)
        let logicalMax = IOHIDElementGetLogicalMax(element)
        print(
            "\(state.identity.prefix) \(name) "
                + "page=\(hexUsage(usagePage)) usage=\(hexUsage(usage)) "
                + "logical=\(logicalMin)...\(logicalMax) value=\(integerValue)"
        )
    }

    func cleanup() {
        guard !isCleanedUp else { return }
        isCleanedUp = true

        interruptSource?.cancel()
        interruptSource = nil
        for state in devices.values {
            print("\(state.identity.prefix) closing")
            IOHIDDeviceClose(state.device, IOOptionBits(kIOHIDOptionsTypeNone))
        }
        devices.removeAll()
        IOHIDManagerUnscheduleFromRunLoop(manager, runLoop, CFRunLoopMode.defaultMode.rawValue)
        IOHIDManagerClose(manager, IOOptionBits(kIOHIDOptionsTypeNone))
    }

    private func printPermissionHintOnce() {
        guard !didPrintPermissionHint else { return }
        didPrintPermissionHint = true
        printInputMonitoringHint()
    }
}

private func usageName(page: UInt32, usage: UInt32) -> String? {
    if page == UInt32(genericDesktopPage) {
        switch usage {
        case 0x30: return "X"
        case 0x31: return "Y"
        case 0x32: return "Z"
        case 0x33: return "Rx"
        case 0x39: return "Hat"
        default: return nil
        }
    }
    if page == UInt32(simulationControlsPage) {
        switch usage {
        case 0xc5: return "LeftBrake"
        case 0xc4: return "RightAccelerator"
        default: return nil
        }
    }
    if page == UInt32(buttonPage) {
        return "Button \(usage)"
    }
    return nil
}

private func hexUsage(_ value: UInt32) -> String {
    String(format: "0x%02X", value)
}

private func monitor(from context: UnsafeMutableRawPointer?) -> HIDMonitor? {
    guard let context else { return nil }
    return Unmanaged<HIDMonitor>.fromOpaque(context).takeUnretainedValue()
}

private func deviceMatchedCallback(
    context: UnsafeMutableRawPointer?,
    result: IOReturn,
    sender: UnsafeMutableRawPointer?,
    device: IOHIDDevice
) {
    _ = sender
    monitor(from: context)?.deviceMatched(device, result: result)
}

private func deviceRemovedCallback(
    context: UnsafeMutableRawPointer?,
    result: IOReturn,
    sender: UnsafeMutableRawPointer?,
    device: IOHIDDevice
) {
    _ = sender
    monitor(from: context)?.deviceRemoved(device, result: result)
}

private func inputValueCallback(
    context: UnsafeMutableRawPointer?,
    result: IOReturn,
    sender: UnsafeMutableRawPointer?,
    value: IOHIDValue
) {
    _ = sender
    monitor(from: context)?.inputValue(value, result: result)
}

let manager: IOHIDManager = IOHIDManagerCreate(kCFAllocatorDefault, IOOptionBits(kIOHIDOptionsTypeNone))
let monitorInstance = HIDMonitor(manager: manager, runLoop: CFRunLoopGetCurrent())
let openResult = monitorInstance.start()
if openResult != kIOReturnSuccess {
    monitorInstance.cleanup()
    exit(EXIT_FAILURE)
}

CFRunLoopRun()
monitorInstance.cleanup()
print("Stopped.")
