from __future__ import annotations

import json
import unittest
from urllib.request import Request, urlopen

from robo_control.config import load_config
from robo_control.simulation import SimulationEngine
from robo_control.server import serve_in_thread


class ServerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.engine = SimulationEngine(load_config())
        self.server, self.thread = serve_in_thread(self.engine, "127.0.0.1", 0)
        self.base_url = f"http://127.0.0.1:{self.server.server_address[1]}"

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.engine.close()
        self.thread.join(timeout=2.0)

    def test_state_endpoint(self) -> None:
        with urlopen(f"{self.base_url}/api/state", timeout=2.0) as response:
            payload = json.load(response)
        self.assertEqual("ready", payload["status"])
        self.assertEqual(6, len(payload["robots"]))

    def test_start_and_pause_actions(self) -> None:
        request = Request(f"{self.base_url}/api/mission/start", method="POST")
        with urlopen(request, timeout=2.0) as response:
            self.assertTrue(json.load(response)["ok"])
        request = Request(f"{self.base_url}/api/mission/pause", method="POST")
        with urlopen(request, timeout=2.0) as response:
            payload = json.load(response)
        self.assertEqual("paused", payload["state"]["status"])


if __name__ == "__main__":
    unittest.main()

