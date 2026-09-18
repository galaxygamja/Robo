"""Localhost-only integration of the production UDP client (no real robot)."""
import json
import socket
import threading
import time
import unittest

from robo_control.hardware import HardwareConfig, UdpFleetTransport, default_profile


class LocalReceiver:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(.05)
        self.address = self.socket.getsockname()
        self.done = threading.Event()
        self.messages = []
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.done.is_set():
            try:
                data, address = self.socket.recvfrom(1025)
            except socket.timeout:
                continue
            except OSError:
                return
            request = json.loads(data)
            self.messages.append(request)
            reply = dict(protocol="robo-hw", version=1, type="response", robot_id="H1",
                request_id=request["request_id"], boot_id="local_test_boot", permit="permit_"+request["request_id"],
                state="disarmed" if request["type"] in {"hello", "stop", "estop"} else "armed",
                accepted=True, reason="test_receiver", seq=request.get("seq", 0), lease_remaining_ms=250,
                left_pwm=request.get("left_pwm", 0), right_pwm=request.get("right_pwm", 0),
                servo_us=request.get("servo_us", [0, 0]), disc_present=False, hardware_enabled=True,
                uptime_ms=1000)
            self.socket.sendto(json.dumps(reply).encode(), address)

    def close(self):
        self.done.set()
        self.thread.join(.2)
        self.socket.close()


class LocalUdpTests(unittest.TestCase):
    def test_real_datagrams_handshake_bench_telemetry_and_stop(self):
        receiver = LocalReceiver()
        data = default_profile()
        robot = data["robots"]["H1"]
        robot.update(host=receiver.address[0], port=receiver.address[1], wiring_verified=True)
        data["robots"], data["network_confirmed"] = {"H1": robot}, True
        client = UdpFleetTransport(HardwareConfig.from_dict(data), "test_token_for_localhost_only_1234")
        try:
            client.connect(time.monotonic())
            deadline = time.monotonic()+2
            while not client.ready and not client.fault and time.monotonic() < deadline:
                client.poll(time.monotonic())
                time.sleep(.002)
            self.assertTrue(client.ready, client.fault)
            self.assertTrue(client.bench("H1", 30, -30, [0, 0], time.monotonic()))
            while not client.ready and not client.fault and time.monotonic() < deadline:
                client.poll(time.monotonic())
                time.sleep(.002)
            self.assertTrue(client.ready, client.fault)
            snapshot = client.snapshot()["robots"]["H1"]
            self.assertEqual(snapshot["telemetry"]["right_pwm"], -30)
            self.assertFalse(snapshot["telemetry"]["disc_present"])
            self.assertLessEqual(snapshot["request_sent_at_s"], snapshot["received_at_s"])
            client.stop("test_complete")
            deadline = time.monotonic()+.2
            while not any(m["type"] == "stop" for m in receiver.messages) and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertTrue(any(m["type"] == "stop" for m in receiver.messages))
            self.assertFalse(client.ready)
        finally:
            client.close()
            receiver.close()
