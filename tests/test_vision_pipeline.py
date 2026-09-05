from __future__ import annotations

import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from robo_control.adapters import CameraFrame
from robo_control.vision.calibration import CalibrationError, FieldCalibration
from robo_control.vision.pipeline import FrameProcessor, FrameRejected

try:
    import cv2  # noqa: F401
    import numpy as np
except ImportError:
    np = None


@unittest.skipIf(np is None, "optional vision dependencies are not installed")
class FrameProcessorTests(unittest.TestCase):
    def setUp(self) -> None:
        # The fixture's field and pixel rectangles coincide. An identity warp
        # gives an independent, exact image oracle for the successful path.
        self.calibration = FieldCalibration(
            image_size_px=(21, 11),
            corners_px=((0, 0), (20, 0), (20, 10), (0, 10)),
            field_size_mm=(20, 10),
        )
        self.image = np.arange(11 * 21 * 3, dtype=np.uint8).reshape(11, 21, 3)
        self.frame = CameraFrame(
            image=self.image,
            captured_at_s=10.0,
            received_at_s=10.025,
            sequence=1,
            source_name="camera:0:session-a",
        )

    def processor(self, *times: float, max_age_s: float = 0.25) -> FrameProcessor:
        return FrameProcessor(
            self.calibration, max_age_s=max_age_s, clock=Mock(side_effect=times)
        )

    def assert_rejected(self, processor, frame, reason: str) -> None:
        with self.assertRaises(FrameRejected) as context:
            processor.process(frame)
        self.assertEqual(reason, context.exception.reason)

    def test_valid_frame_preserves_metadata_and_leaves_input_image_unchanged(self) -> None:
        original_pixels = self.image.copy()
        result = self.processor(10.05, 10.125).process(self.frame)

        self.assertIs(self.frame, result.original)
        self.assertEqual(10.125, result.processed_at_s)
        self.assertAlmostEqual(0.125, result.age_s)
        np.testing.assert_array_equal(original_pixels, result.image)
        np.testing.assert_array_equal(original_pixels, self.image)
        self.assertFalse(np.shares_memory(result.image, self.image))
        result.image[:] = 0
        np.testing.assert_array_equal(original_pixels, self.image)

    def test_legacy_frame_without_received_timestamp_remains_usable(self) -> None:
        legacy = CameraFrame(self.image, 10.0, 1, "legacy-camera")
        result = self.processor(10.05, 10.125).process(legacy)
        self.assertIs(legacy, result.original)
        self.assertAlmostEqual(0.125, result.age_s)

    def test_age_counts_from_read_start_not_receive_time(self) -> None:
        slow_capture = replace(self.frame, received_at_s=10.2)
        self.assert_rejected(
            self.processor(10.3), slow_capture, "stale_frame"
        )

    def test_stale_frame_is_rejected_before_warp_runs(self) -> None:
        with patch.object(FieldCalibration, "warp") as warp:
            self.assert_rejected(
                self.processor(10.251), self.frame, "stale_frame"
            )
        warp.assert_not_called()

    def test_frame_expiring_during_warp_is_rejected(self) -> None:
        self.assert_rejected(
            self.processor(10.1, 10.251), self.frame, "stale_frame"
        )

    def test_frame_at_exact_age_limit_is_accepted(self) -> None:
        result = self.processor(10.125, 10.25).process(self.frame)
        self.assertEqual(0.25, result.age_s)

    def test_finish_rejects_frame_that_expires_during_external_work(self) -> None:
        processor = self.processor(10.1, 10.251)

        processor.begin_frame(self.frame)
        with self.assertRaises(FrameRejected) as context:
            processor.finish_frame(self.frame)

        self.assertEqual("stale_frame", context.exception.reason)

    def test_finish_returns_external_work_completion_time_and_frame_age(self) -> None:
        processor = self.processor(10.05, 10.125)

        started_at_s = processor.begin_frame(self.frame)
        processed_at_s, age_s = processor.finish_frame(self.frame)

        self.assertEqual(10.05, started_at_s)
        self.assertEqual(10.125, processed_at_s)
        self.assertAlmostEqual(0.125, age_s)

    def test_abandon_consumes_sequence_but_allows_the_next_frame(self) -> None:
        processor = self.processor(10.05, 10.075, 10.1, 10.125)
        processor.begin_frame(self.frame)
        processor.abandon_frame()

        with self.assertRaises(FrameRejected) as context:
            processor.begin_frame(self.frame)
        self.assertEqual("out_of_order_sequence", context.exception.reason)

        next_frame = replace(
            self.frame, sequence=2, captured_at_s=10.025, received_at_s=10.05
        )
        processor.begin_frame(next_frame)
        processed_at_s, age_s = processor.finish_frame(next_frame)
        self.assertEqual(10.125, processed_at_s)
        self.assertAlmostEqual(0.1, age_s)

    def test_begin_rejects_another_frame_while_one_is_active(self) -> None:
        clock = Mock(side_effect=(10.05,))
        processor = FrameProcessor(
            self.calibration, max_age_s=0.25, clock=clock
        )
        processor.begin_frame(self.frame)

        with self.assertRaisesRegex(RuntimeError, "finish or abandon"):
            processor.begin_frame(replace(self.frame, sequence=2))

        clock.assert_called_once_with()
        processor.abandon_frame()

    def test_finish_rejects_wrong_frame_without_losing_active_frame(self) -> None:
        processor = self.processor(10.05, 10.125)
        processor.begin_frame(self.frame)

        with self.assertRaisesRegex(RuntimeError, "not the active consumed frame"):
            processor.finish_frame(replace(self.frame, sequence=2))

        processed_at_s, age_s = processor.finish_frame(self.frame)
        self.assertEqual(10.125, processed_at_s)
        self.assertAlmostEqual(0.125, age_s)

    def test_future_and_reversed_acquisition_timestamps_are_rejected(self) -> None:
        cases = (
            replace(self.frame, captured_at_s=10.2, received_at_s=10.2),
            replace(self.frame, received_at_s=10.2),
            replace(self.frame, captured_at_s=10.05, received_at_s=10.025),
        )
        for frame in cases:
            with self.subTest(captured=frame.captured_at_s, received=frame.received_at_s):
                self.assert_rejected(self.processor(10.1), frame, "timestamp_order")

    def test_nonfinite_acquisition_timestamps_are_rejected(self) -> None:
        for field in ("captured_at_s", "received_at_s"):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(field=field, value=value):
                    self.assert_rejected(
                        self.processor(10.1), replace(self.frame, **{field: value}),
                        "invalid_timestamp",
                    )

    def test_nonnumeric_acquisition_timestamps_are_rejected(self) -> None:
        for field in ("captured_at_s", "received_at_s"):
            for value in (True, "10.0"):
                with self.subTest(field=field, value=value):
                    self.assert_rejected(
                        self.processor(10.1), replace(self.frame, **{field: value}),
                        "invalid_timestamp",
                    )

    def test_nonfinite_clock_is_rejected_before_or_after_warp(self) -> None:
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(phase="before", value=value):
                self.assert_rejected(
                    self.processor(value), self.frame, "invalid_timestamp"
                )
            # -inf is also a backwards clock, so either reason is a safe
            # rejection; no nonfinite processing time may be returned.
            with self.subTest(phase="after", value=value), self.assertRaises(FrameRejected):
                self.processor(10.1, value).process(self.frame)

    def test_clock_moving_backwards_during_warp_is_rejected(self) -> None:
        self.assert_rejected(
            self.processor(10.125, 10.1), self.frame, "clock_moved_backwards"
        )

    def test_nonnumeric_clock_is_rejected_before_or_after_warp(self) -> None:
        for value in (None, True, "10.1"):
            for times in ((value,), (10.1, value)):
                with self.subTest(times=times):
                    self.assert_rejected(
                        self.processor(*times), self.frame, "invalid_timestamp"
                    )

    def test_duplicate_and_decreasing_sequences_cannot_be_used_again(self) -> None:
        for next_sequence in (2, 1):
            with self.subTest(next_sequence=next_sequence):
                processor = self.processor(10.1, 10.125, 10.15)
                processor.process(replace(self.frame, sequence=2))
                self.assert_rejected(
                    processor, replace(self.frame, sequence=next_sequence),
                    "out_of_order_sequence",
                )

    def test_sequences_must_be_positive_integers(self) -> None:
        for sequence in (0, -1, True, 1.0, "1"):
            with self.subTest(sequence=sequence):
                self.assert_rejected(
                    self.processor(10.1), replace(self.frame, sequence=sequence),
                    "out_of_order_sequence",
                )

    def test_new_sequence_with_older_capture_time_is_rejected(self) -> None:
        processor = self.processor(10.1, 10.125, 10.15)
        processor.process(self.frame)
        self.assert_rejected(
            processor, replace(self.frame, sequence=2, captured_at_s=9.99),
            "out_of_order_timestamp",
        )

    def test_increasing_sequence_and_time_are_processed_in_order(self) -> None:
        processor = self.processor(10.1, 10.125, 10.15, 10.175)
        processor.process(self.frame)
        next_frame = replace(
            self.frame, sequence=2, captured_at_s=10.1, received_at_s=10.125
        )
        result = processor.process(next_frame)
        self.assertIs(next_frame, result.original)
        self.assertAlmostEqual(0.075, result.age_s)

    def test_source_change_requires_a_new_processor(self) -> None:
        processor = self.processor(10.1, 10.125, 10.15)
        processor.process(self.frame)
        other_source = replace(self.frame, sequence=2, source_name="camera:1:session-b")
        self.assert_rejected(processor, other_source, "source_changed")
        result = self.processor(10.15, 10.175).process(other_source)
        self.assertIs(other_source, result.original)

    def test_empty_and_nonstring_sources_are_rejected(self) -> None:
        for source in ("", None, 7):
            with self.subTest(source=source):
                self.assert_rejected(
                    self.processor(10.1), replace(self.frame, source_name=source),
                    "invalid_source",
                )

    def test_resolution_change_requires_recalibration(self) -> None:
        for shape in ((12, 21, 3), (11, 22, 3), (21, 11, 3)):
            with self.subTest(shape=shape):
                wrong_size = replace(self.frame, image=np.zeros(shape, dtype=np.uint8))
                with self.assertRaisesRegex(FrameRejected, "resolution.*recalibrate"):
                    self.processor(10.1).process(wrong_size)

    def test_nonimages_and_unsupported_image_formats_are_rejected(self) -> None:
        bad_images = (
            None,
            {"robots": []},
            [[0, 1], [2, 3]],
            np.zeros((11, 21, 3), dtype=np.float32),
            np.zeros((11, 21, 2), dtype=np.uint8),
            np.zeros((11, 21, 3, 1), dtype=np.uint8),
            np.zeros((21,), dtype=np.uint8),
        )
        for image in bad_images:
            with self.subTest(image_type=type(image), shape=getattr(image, "shape", None)):
                with self.assertRaises(FrameRejected) as context:
                    self.processor(10.1).process(replace(self.frame, image=image))
                self.assertTrue(context.exception.reason.startswith("invalid_image:"))

    def test_frame_rejected_during_warp_cannot_be_retried_with_same_sequence(self) -> None:
        processor = self.processor(10.1, 10.15, 10.175, 10.2)
        with self.assertRaises(FrameRejected):
            processor.process(replace(self.frame, image=None))
        self.assert_rejected(processor, self.frame, "out_of_order_sequence")
        result = processor.process(replace(self.frame, sequence=2))
        self.assertEqual(2, result.original.sequence)

    def test_postwarp_expired_frame_cannot_be_retried_as_new_input(self) -> None:
        processor = self.processor(10.1, 10.26, 10.27, max_age_s=0.25)
        self.assert_rejected(processor, self.frame, "stale_frame")
        # Increasing the age allowance does not undo consumed sequence state.
        processor.max_age_s = 1.0
        self.assert_rejected(processor, self.frame, "out_of_order_sequence")

    def test_invalid_processor_limits_are_rejected_at_construction(self) -> None:
        for argument in ("max_age_s", "pixels_per_mm"):
            for value in (0, -1, True, float("nan"), float("inf"), "1"):
                with (
                    self.subTest(argument=argument, value=value),
                    self.assertRaises(CalibrationError),
                ):
                    FrameProcessor(self.calibration, **{argument: value})


if __name__ == "__main__":
    unittest.main()
