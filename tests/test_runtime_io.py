from __future__ import annotations

import io
import json
import multiprocessing
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from robo_control.runtime_io import AsyncJsonlReport, LatestRecordMailbox


class MailboxTests(unittest.TestCase):
    def setUp(self):
        self.mailbox = LatestRecordMailbox(multiprocessing.get_context("spawn"), capacity=1024)

    def test_overwrite_is_bounded_and_only_newest_revision_is_delivered(self):
        self.assertIsNone(self.mailbox.poll())
        for sequence in range(1, 101):
            self.assertTrue(self.mailbox.publish({"sequence": sequence, "name": "비버"}))
        revision, record = self.mailbox.poll()
        self.assertEqual(100, revision)
        self.assertEqual({"sequence": 100, "name": "비버"}, record)
        self.assertIsNone(self.mailbox.poll(revision))
        self.assertEqual(1024, len(self.mailbox.buffer))

    def test_locked_writer_does_not_block_polls_or_another_write(self):
        self.mailbox.lock.acquire()
        try:
            start = time.monotonic()
            self.assertIsNone(self.mailbox.poll())
            self.assertFalse(self.mailbox.publish({"sequence": 1}))
            self.assertLess(time.monotonic() - start, .05)
        finally:
            self.mailbox.lock.release()

    def test_nonfinite_or_oversized_json_never_replaces_last_good_message(self):
        self.mailbox.publish({"sequence": 1})
        for value in ({"data": "x" * 1024}, {"data": float("nan")}, {"data": float("inf")}):
            with self.subTest(value=str(value)[:50]), self.assertRaises(ValueError):
                self.mailbox.publish(value)
        self.assertEqual((1, {"sequence": 1}), self.mailbox.poll())


class ReportTests(unittest.TestCase):
    def test_ordered_unicode_jsonl_flush_and_exclusive_creation(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "상태.jsonl"
            writer = AsyncJsonlReport(path)
            for i in range(20):
                self.assertTrue(writer.submit({"sequence": i, "robot": "햄스터"}))
            self.assertTrue(writer.finish())
            rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(list(range(20)), [row["sequence"] for row in rows])
            with self.assertRaises(FileExistsError):
                AsyncJsonlReport(path)
            self.assertFalse(writer.submit({"too": "late"}))

    def test_slow_disk_never_blocks_submit_and_full_queue_is_explicit(self):
        entered, release = threading.Event(), threading.Event()

        class SlowFile(io.StringIO):
            def write(self, value):
                entered.set()
                release.wait(2.)
                return super().write(value)

        with tempfile.TemporaryDirectory() as folder:
            with patch.object(Path, "open", return_value=SlowFile()):
                writer = AsyncJsonlReport(Path(folder) / "blocked.jsonl", capacity=2)
            try:
                self.assertTrue(writer.submit({"first": True}))
                self.assertTrue(entered.wait(1.))
                start = time.monotonic()
                self.assertTrue(writer.submit({"second": True}))
                self.assertTrue(writer.submit({"third": True}))
                self.assertFalse(writer.submit({"overflow": True}))
                self.assertLess(time.monotonic() - start, .05)
                self.assertIn("backpressure", writer.error)
            finally:
                release.set()
                self.assertFalse(writer.finish())
            self.assertFalse(writer.thread.is_alive())

    def test_disk_failure_is_visible_to_supervisor(self):
        class BrokenFile(io.StringIO):
            def write(self, value):
                raise OSError("disk full")

        with tempfile.TemporaryDirectory() as folder:
            with patch.object(Path, "open", return_value=BrokenFile()):
                writer = AsyncJsonlReport(Path(folder) / "broken.jsonl")
            writer.submit({"event": "test"})
            self.assertFalse(writer.finish())
            self.assertIn("disk full", writer.error)


if __name__ == "__main__":
    unittest.main()
