from __future__ import annotations

import contextlib
import io
import json
import sqlite3
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from test_manipulator_wire import operation

from robo_control.fake_manipulator import FakeManipulatorReceiver
from robo_control.journal_sender import JournaledManipulatorSender
from robo_control.operation_journal import (
    ACCEPTED_ACK,
    EXECUTE_INTENT,
    MAX_OPERATIONS,
    STOP_ACK,
    STOP_REQUESTED,
    JournalError,
    OperationJournal,
    main,
)
from robo_control.wire_codec import encode_frame


class JournalFixture:
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.path = Path(temporary.name) / "operations.sqlite3"
        self.journal = self.open_new()
        self.identity = self.journal.journal_id

    def open_new(self):
        journal = OperationJournal.create(self.path, robot_ids=["H1", "H2", "B1", "B2"])
        self.addCleanup(journal.close)
        return journal

    def reopen(self):
        journal = OperationJournal.open(self.path, expected_id=self.identity)
        self.addCleanup(journal.close)
        return journal

    def reserve(self, journal=None, **changes):
        args = {"work_id": "match1-D1-close", "robot_id": "H1", "session_id": "host1", "operation": operation()}
        args.update(changes)
        (journal or self.journal).reserve(**args)

    def quarantine(self, journal=None, **changes):
        args = {"reviewed_by": "TEST-OPERATOR", "evidence": "Synthetic endpoint and old sender stopped; outcome not asserted",
                "operator_confirmed_stopped": True}
        args.update(changes)
        (journal or self.journal).quarantine("match1-D1-close", **args)

    def mark(self, flag, journal=None):
        (journal or self.journal).mark("match1-D1-close", robot_id="H1", session_id="host1", flag=flag)


class JournalTests(JournalFixture, unittest.TestCase):

    def test_create_requires_explicit_fleet(self):
        for fleet in ([], ["H1", "H1"], ["bad id"], "H1"):
            with self.assertRaises(JournalError):
                OperationJournal.create(self.path.parent / "new.db", robot_ids=fleet)

    def test_create_never_overwrites(self):
        original = self.path.read_bytes()
        with self.assertRaises(FileExistsError):
            self.open_new()
        self.assertEqual(original, self.path.read_bytes())

    def test_missing_journal_is_not_created(self):
        missing = self.path.parent / "missing.db"
        with self.assertRaises(JournalError):
            OperationJournal.open(missing, expected_id=self.identity)
        self.assertFalse(missing.exists())

    def test_wrong_identity_refused_without_modification(self):
        original = self.path.read_bytes()
        with self.assertRaises(JournalError):
            OperationJournal.open(self.path, expected_id="other")
        self.assertEqual(original, self.path.read_bytes())

    def test_reservation_survives_close_and_reopen(self):
        self.reserve()
        self.journal.close()
        row = self.reopen().snapshot()["attempts"][0]
        self.assertEqual("unresolved", row["state"])
        self.assertEqual(0, row["flags"])
        self.assertIsNone(row["physical_success"])

    def test_restart_new_session_and_work_id_still_blocks_robot(self):
        self.reserve()
        with self.assertRaisesRegex(JournalError, "unresolved"):
            self.reserve(self.reopen(), session_id="newhost", work_id="renamed", operation=operation(command_id="renamed"))

    def test_work_identity_prevents_reassignment_to_other_robot(self):
        self.reserve()
        self.quarantine()
        with self.assertRaises(JournalError):
            self.reserve(robot_id="B2", session_id="other", operation=operation(command_id="other"))

    def test_quarantine_never_allows_same_work_again(self):
        self.reserve()
        self.quarantine()
        with self.assertRaises(JournalError):
            self.reserve(session_id="other")

    def test_distinct_work_after_explicit_quarantine_is_allowed(self):
        self.reserve()
        self.quarantine()
        self.reserve(work_id="match1-next-work", operation=operation(command_id="next"))
        rows = self.reopen().snapshot()["attempts"]
        self.assertEqual(["quarantined_never_retry", "unresolved"], [r["state"] for r in rows])
        self.assertTrue(all(r["physical_success"] is None for r in rows))

    def test_command_identity_cannot_be_renamed_in_same_session(self):
        self.reserve()
        self.quarantine()
        with self.assertRaises(JournalError):
            self.reserve(work_id="other")

    def test_four_robots_have_independent_outstanding_work(self):
        for rid in self.journal.fleet:
            self.reserve(robot_id=rid, work_id="work-" + rid)
        self.assertEqual(4, len(self.journal.snapshot()["attempts"]))

    def test_operator_confirmation_and_written_evidence_required(self):
        self.reserve()
        for changes in ({"operator_confirmed_stopped": False}, {"operator_confirmed_stopped": 1},
                        {"reviewed_by": ""}, {"evidence": " "}, {"evidence": "x" * 1025}):
            with self.assertRaises(JournalError):
                self.quarantine(**changes)
        self.assertEqual("unresolved", self.journal.snapshot()["attempts"][0]["state"])

    def test_stop_ack_does_not_resolve_physical_outcome(self):
        self.reserve()
        for flag in (EXECUTE_INTENT, ACCEPTED_ACK, STOP_REQUESTED, STOP_ACK):
            self.mark(flag)
        row = self.reopen().snapshot()["attempts"][0]
        self.assertEqual(15, row["flags"])
        self.assertEqual("unresolved", row["state"])
        self.assertIsNone(row["physical_success"])

    def test_ack_cannot_precede_execute_or_stop_intent(self):
        self.reserve()
        for flag in (ACCEPTED_ACK, STOP_ACK):
            with self.assertRaises(JournalError):
                self.mark(flag, self.reopen())

    def test_execute_marker_is_consumed_once(self):
        self.reserve()
        self.mark(EXECUTE_INTENT)
        with self.assertRaises(JournalError):
            self.mark(EXECUTE_INTENT)

    def test_quarantine_fences_stale_sender_execute(self):
        self.reserve()
        stale = self.reopen()
        self.quarantine()
        with self.assertRaises(JournalError):
            self.mark(EXECUTE_INTENT, stale)

    def test_wrong_session_cannot_mark(self):
        self.reserve()
        with self.assertRaises(JournalError):
            self.journal.mark("match1-D1-close", robot_id="H1", session_id="wrong", flag=EXECUTE_INTENT)

    def test_readonly_inspection_preserves_bytes(self):
        self.reserve()
        before = self.path.read_bytes()
        snapshot = OperationJournal.inspect_file(self.path)
        self.assertFalse(snapshot["automatic_replay_enabled"])
        self.assertEqual(before, self.path.read_bytes())

    def test_readonly_handle_cannot_mutate(self):
        with (OperationJournal._open(self.path, expected_id=None, writable=False) as journal,
              self.assertRaises(JournalError)):
            self.reserve(journal)

    def test_corrupt_or_empty_database_fails_closed(self):
        for content in (b"", b"not sqlite", b"SQLite format 3\0garbage"):
            path = self.path.parent / "corrupt.db"
            path.write_bytes(content)
            with self.assertRaises(JournalError):
                OperationJournal.open(path, expected_id=self.identity)

    def test_unrecognized_schema_does_not_run_triggers(self):
        with contextlib.closing(sqlite3.connect(self.path, isolation_level=None)) as db:
            db.execute("CREATE TABLE unrelated (value TEXT)")
        before = self.path.read_bytes()
        with self.assertRaises(JournalError):
            self.reopen()
        self.assertEqual(before, self.path.read_bytes())

    def test_invalid_stored_operation_flags_or_review_rejected(self):
        self.reserve()
        for sql, value in (("operation", '{"broken":true}'), ("operation", b"blob"), ("flags", 2), ("reviewer", "")):
            with contextlib.closing(sqlite3.connect(self.path, isolation_level=None)) as db:
                original = db.execute(f"SELECT {sql} FROM attempts").fetchone()[0]
                db.execute(f"UPDATE attempts SET {sql}=?", (value,))
            with self.assertRaises(JournalError):
                self.reopen()
            with contextlib.closing(sqlite3.connect(self.path, isolation_level=None)) as db:
                db.execute(f"UPDATE attempts SET {sql}=?", (original,))

    def test_bounded_history_never_evicts(self):
        self.reserve()
        self.quarantine()
        with patch("robo_control.operation_journal.MAX_OPERATIONS", 1), self.assertRaises(JournalError):
            self.reserve(work_id="next", operation=operation(command_id="next"))
        self.assertEqual(1, len(self.reopen().snapshot()["attempts"]))
        self.assertEqual(4096, MAX_OPERATIONS)

    def test_lock_contention_never_proceeds_unjournaled(self):
        with contextlib.closing(sqlite3.connect(self.path, isolation_level=None)) as blocker:
            blocker.execute("BEGIN IMMEDIATE")
            with self.assertRaises(JournalError):
                self.reserve()
            blocker.execute("ROLLBACK")
        self.assertEqual([], self.reopen().snapshot()["attempts"])

    def test_committed_reservation_survives_abrupt_process_exit(self):
        code = """import os, sys
from robo_control.operation_journal import OperationJournal
j = OperationJournal.open(sys.argv[1], expected_id=sys.argv[2])
j.reserve(work_id='crash-work', robot_id='H1', session_id='crashed', operation={
    'command_id':'cmd', 'phase':'close_servo', 'action':'disc_latch_close', 'timeout_ms':1000})
os._exit(17)
"""
        result = subprocess.run([sys.executable, "-c", code, str(self.path), self.identity], timeout=20, check=False)
        self.assertEqual(17, result.returncode)
        row = self.reopen().snapshot()["attempts"][0]
        self.assertEqual("crash-work", row["work_id"])
        with self.assertRaises(JournalError):
            self.reserve(self.reopen(), work_id="new-work", session_id="new-host")

    def test_uncommitted_interrupted_transaction_is_rolled_back(self):
        code = """import os, sqlite3, sys
db=sqlite3.connect(sys.argv[1], isolation_level=None)
db.execute('BEGIN IMMEDIATE')
db.execute('UPDATE metadata SET version=99')
os._exit(18)
"""
        result = subprocess.run([sys.executable, "-c", code, str(self.path)], timeout=20, check=False)
        self.assertEqual(18, result.returncode)
        self.assertEqual([], self.reopen().snapshot()["attempts"])

    def test_two_processes_cannot_reserve_two_commands_for_same_robot(self):
        code = """import sys
from robo_control.operation_journal import OperationJournal, JournalError
try:
    with OperationJournal.open(sys.argv[1], expected_id=sys.argv[2]) as journal:
        journal.reserve(work_id=sys.argv[3], robot_id='H1', session_id=sys.argv[3], operation={
            'command_id':sys.argv[3], 'phase':'close_servo', 'action':'disc_latch_close', 'timeout_ms':1000})
except JournalError:
    sys.exit(2)
"""
        processes = [subprocess.Popen([sys.executable, "-c", code, str(self.path), self.identity, f"process-{i}"])
                     for i in range(2)]
        try:
            codes = sorted(p.wait(timeout=20) for p in processes)
        finally:
            for process in processes:
                if process.poll() is None:
                    process.kill()
                process.wait()
        self.assertEqual([0, 2], codes)
        self.assertEqual(1, len(self.reopen().snapshot()["attempts"]))

    def test_cli_inspect_and_quarantine_never_report_success(self):
        self.reserve()
        with contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(0, main(["inspect", str(self.path)]))
        self.assertIsNone(json.loads(output.getvalue())["physical_success"])
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(0, main(["quarantine", str(self.path), "--journal-id", self.identity,
                "--work-id", "match1-D1-close", "--reviewed-by", "TEST", "--evidence", "test-only stopped",
                "--confirm-devices-and-old-process-stopped"]))
        self.assertEqual("quarantined_never_retry", self.reopen().snapshot()["attempts"][0]["state"])


class JournalSenderTests(JournalFixture, unittest.TestCase):
    def ready(self):
        self.sender = JournaledManipulatorSender("H1", session_id="host1", journal=self.journal)
        self.receiver = FakeManipulatorReceiver("H1", supported_actions={"gripper_close"})
        message = self.sender.begin(operation(), 10., work_id="match1-D1-close")
        self.assertEqual(0, self.reopen().snapshot()["attempts"][0]["flags"])
        self.assertTrue(self.exchange(message, .01))

    def exchange(self, message, at):
        return self.sender.accept_frame(self.receiver.receive_frame(encode_frame(message), 1000. + at), 10. + at)

    def test_execute_intent_is_committed_before_message_handoff(self):
        self.ready()
        message = self.sender.execute(10.02)
        self.assertEqual(EXECUTE_INTENT, self.reopen().snapshot()["attempts"][0]["flags"])
        self.assertEqual(0, self.receiver.accepted_operations)
        self.assertTrue(self.exchange(message, .03))
        self.assertEqual(3, self.reopen().snapshot()["attempts"][0]["flags"])

    def test_stop_ack_and_reboot_cannot_unblock_robot(self):
        self.ready()
        self.exchange(self.sender.execute(10.02), .03)
        self.exchange(self.sender.stop(10.04), .05)
        another = JournaledManipulatorSender("H1", session_id="restart", journal=self.reopen())
        with self.assertRaises(JournalError):
            another.begin(operation(command_id="new"), 0., work_id="renamed")
        self.assertEqual(15, self.reopen().snapshot()["attempts"][0]["flags"])

    def test_lost_ack_never_replayed_after_restart(self):
        self.ready()
        message = self.sender.execute(10.02)
        self.receiver.receive(message, 1000.03)
        self.sender.poll(10.33)
        another = JournaledManipulatorSender("H1", session_id="restart", journal=self.reopen())
        with self.assertRaises(JournalError):
            another.begin(operation(), 0., work_id="match1-D1-close")
        self.assertEqual(1, self.receiver.accepted_operations)

    def test_disk_failure_prevents_execute_handoff(self):
        self.ready()
        with patch.object(self.journal, "mark", side_effect=JournalError("disk full")), self.assertRaises(JournalError):
            self.sender.execute(10.02)
        self.assertEqual(0, self.receiver.accepted_operations)
        self.assertEqual("closed", self.sender.snapshot()["state"])
        self.assertIsNotNone(self.sender.snapshot()["storage_fault"])

    def test_storage_failure_does_not_withhold_stop(self):
        self.ready()
        self.exchange(self.sender.execute(10.02), .03)
        with patch.object(self.journal, "mark", side_effect=JournalError("disk full")) as mark:
            stop = self.sender.stop(10.04)
        mark.assert_not_called()
        self.assertEqual("stop", stop["type"])
        self.receiver.receive(stop, 1000.05)
        self.assertIsNone(self.receiver.snapshot()["requested_action"])

    def test_acceptance_persistence_failure_blocks_further_nonstop_requests(self):
        self.ready()
        message = self.sender.execute(10.02)
        with patch.object(self.journal, "mark", side_effect=JournalError("commit failed")):
            self.assertFalse(self.exchange(message, .03))
        with self.assertRaises(JournalError):
            self.sender.sample(10.04)
        self.assertIsNotNone(self.sender.stop(10.04))

    def test_quarantined_prepared_sender_cannot_execute(self):
        self.ready()
        self.quarantine(self.reopen())
        with self.assertRaises(JournalError):
            self.sender.execute(10.02)
        self.assertEqual(0, self.receiver.accepted_operations)

    def test_begin_storage_failure_does_not_return_hello(self):
        sender = JournaledManipulatorSender("H1", session_id="host1", journal=self.journal)
        with patch.object(self.journal, "reserve", side_effect=JournalError("disk full")), self.assertRaises(JournalError):
            sender.begin(operation(), 10., work_id="work")
        self.assertEqual("idle", sender.snapshot()["state"])


if __name__ == "__main__":
    unittest.main()
