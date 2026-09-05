from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from tools.generate_robot_tags import main


@unittest.skipUnless(cv2 is not None and np is not None and hasattr(cv2, "aruco"),
                     "install the vision extra")
class TagAssetTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.config = self.root / "tags.json"
        self.config.write_text(json.dumps({
            "schema_version": 1,
            "dictionary_name": "DICT_APRILTAG_36h11",
            "tag_to_robot": {"0": "H1", "1": "H2", "2": "B1", "3": "B2"},
            "tag_size_mm": 45.0,
        }))

    def invoke(self, *args):
        output, errors = io.StringIO(), io.StringIO()
        with redirect_stdout(output), redirect_stderr(errors):
            status = main([str(value) for value in args])
        return status, output.getvalue(), errors.getvalue()

    def test_generated_assets_match_detector_dictionary_and_forward_axis(self):
        output = self.root / "print"
        status, text, errors = self.invoke("--config", self.config, "--output-dir", output,
                                           "--side-px", 160, "--quiet-zone-px", 30)
        self.assertEqual(0, status, errors)
        self.assertEqual(4, json.loads(text)["tag_count"])
        manifest = json.loads((output / "manifest.json").read_text())
        self.assertEqual("canonical_corner_0_to_1", manifest["marker_forward"])
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        detector = cv2.aruco.ArucoDetector(dictionary)
        for item in manifest["files"]:
            printable = cv2.imdecode(
                np.frombuffer((output / item["printable_marker_png"]).read_bytes(), np.uint8),
                cv2.IMREAD_GRAYSCALE,
            )
            corners, ids, _ = detector.detectMarkers(printable)
            with self.subTest(tag_id=item["tag_id"]):
                self.assertEqual((220, 220), printable.shape)
                self.assertTrue(np.all(printable[:30, :] == 255))
                self.assertTrue(np.all(printable[-30:, :] == 255))
                self.assertTrue(np.all(printable[:, :30] == 255))
                self.assertTrue(np.all(printable[:, -30:] == 255))
                self.assertEqual([item["tag_id"]], ids.reshape(-1).tolist())
                points = corners[0].reshape(4, 2)
                self.assertGreater(points[1, 0] - points[0, 0], 150)
                self.assertAlmostEqual(points[1, 1], points[0, 1], delta=1)
                self.assertTrue((output / item["front_reference_png"]).is_file())

    def test_existing_asset_is_not_partially_overwritten_without_opt_in(self):
        output = self.root / "print"
        self.assertEqual(0, self.invoke("--config", self.config, "--output-dir", output)[0])
        manifest = (output / "manifest.json").read_bytes()
        status, _, errors = self.invoke("--config", self.config, "--output-dir", output)
        self.assertEqual(2, status)
        self.assertIn("refusing to overwrite", errors)
        self.assertEqual(manifest, (output / "manifest.json").read_bytes())


if __name__ == "__main__":
    unittest.main()
