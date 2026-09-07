from __future__ import annotations

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from test_runtime_session import detection, make_session, zero_output

from robo_control.runtime import supervise


class SupervisorFailureTests(unittest.TestCase):
    """Deterministic faults in the parent, complementing real child-process tests."""

    def setUp(self):
        self.session = make_session()
        for seq in (1, 2, 3):
            stamp = 10. + (seq - 1) * .05
            self.session.advance(detection(seq, stamp), stamp)
        self.clock = SimpleNamespace(now=10.12)
        self.state = SimpleNamespace(value=1)
        self.worker = Mock()
        self.worker.is_alive.return_value = True
        self.worker.join.side_effect = lambda **_: setattr(self.worker.is_alive, "return_value", False)
        self.stop = threading.Event()
        self.mailbox = Mock()
        self.mailbox.poll.return_value = None
        self.report = Mock(error=None)
        self.rows = []

        def submit(row):
            self.rows.append(row)
            self.stop.set()  # one parent iteration, without a busy loop
            return True

        self.report.submit.side_effect = submit
        self.report.finish.return_value = True

    def run_supervisor(self):
        with patch("robo_control.runtime.time.monotonic", side_effect=lambda: self.clock.now):
            summary = supervise(self.session, self.worker, self.mailbox, self.stop,
                self.state, self.report, duration_s=1., startup_timeout_s=.5)
        self.assertTrue(zero_output(summary["final"]))
        self.assertEqual(1, sum(row["status"] == "closed" for row in self.rows))
        self.assertTrue(summary["worker_stopped"])
        return summary

    def test_scheduler_gap_emits_only_one_terminal_record(self):
        self.clock.now = 10.22
        result = self.run_supervisor()
        self.assertEqual("supervisor_deadline_missed", result["status"])
        self.assertEqual(1, len(self.rows))

    def test_computation_overrun_stops_before_nonzero_event_is_submitted(self):
        advance = self.session.advance

        def slow_advance(record, now):
            event = advance(record, now)
            self.clock.now += .11
            return event

        self.session.advance = slow_advance
        result = self.run_supervisor()
        self.assertEqual("supervisor_deadline_missed", result["status"])
        self.assertTrue(all(zero_output(row) for row in self.rows))

    def test_measurement_expiring_during_calculation_is_zeroed_before_logging(self):
        self.session.advance(None, 10.20)
        self.clock.now = 10.26
        self.mailbox.poll.return_value = (1, detection(4, 10.11))
        advance = self.session.advance

        def aging_advance(record, now):
            event = advance(record, now)
            self.clock.now += .06
            return event

        self.session.advance = aging_advance
        result = self.run_supervisor()
        self.assertEqual("operator_stop", result["status"])
        self.assertEqual("observation_watchdog", self.rows[0]["status"])
        self.assertTrue(all(zero_output(row) for row in self.rows))

    def test_report_backpressure_stops_and_summary_retains_terminal_event(self):
        def failed_submit(row):
            self.rows.append(row)
            self.report.error = "injected_full_queue"
            return False

        self.report.submit.side_effect = failed_submit
        self.report.finish.return_value = False
        result = self.run_supervisor()
        self.assertEqual("report_failed", result["status"])
        self.assertFalse(result["report_complete"])
        self.assertEqual("injected_full_queue", result["report_error"])

    def test_unexpected_controller_error_stops_active_output_before_cleanup(self):
        self.session.advance = Mock(side_effect=ValueError("injected computation failure"))
        result = self.run_supervisor()
        self.assertEqual("runtime_failed", result["status"])
        self.assertIn("injected computation failure", result["error"])

    def test_worker_decode_failure_flag_is_not_a_successful_eof(self):
        self.state.value = 5
        result = self.run_supervisor()
        self.assertEqual("video_decode_failed_or_unverified_end", result["status"])


if __name__ == "__main__":
    unittest.main()
