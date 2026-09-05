from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from robo_control.adapters import (
    CameraFrame,
    OpenCVCameraSource,
    SyntheticCameraSource,
    VideoFileSource,
)


class CaptureError(Exception):
    pass


class FakeCapture:
    def __init__(self, frames=(), *, opened=True, media_positions=()) -> None:
        self.frames = iter(frames)
        self.opened = opened
        self.media_positions = iter(media_positions)
        self.read_count = 0
        self.release_count = 0
        self.get_properties = []

    def isOpened(self):
        if isinstance(self.opened, Exception):
            raise self.opened
        return self.opened

    def read(self):
        self.read_count += 1
        result = next(self.frames, (False, None))
        if isinstance(result, Exception):
            raise result
        return result

    def release(self):
        self.release_count += 1

    def get(self, property_id):
        self.get_properties.append(property_id)
        result = next(self.media_positions, 0.0)
        if isinstance(result, Exception):
            raise result
        return result


def fake_cv2(capture: FakeCapture):
    return SimpleNamespace(VideoCapture=Mock(return_value=capture), error=CaptureError, CAP_PROP_POS_MSEC=0)


class CameraSourceTests(unittest.TestCase):
    def test_original_four_argument_frame_contract_is_preserved(self) -> None:
        image = object()
        frame = CameraFrame(image, 1.5, 1, "legacy")
        self.assertIs(image, frame.image)
        self.assertIsNone(frame.received_at_s)
        self.assertIsNone(frame.media_time_s)
        self.assertEqual("host_read_start", frame.timestamp_basis)
        self.assertFalse(frame.is_replay)

    def test_synthetic_source_identifies_timestamp_and_keeps_sequence(self) -> None:
        snapshot = {"robots": []}
        with patch("robo_control.adapters.time.monotonic", return_value=10.0):
            source = SyntheticCameraSource(lambda: snapshot)
            first, second = source.read(), source.read()
        self.assertIs(snapshot, first.image)
        self.assertEqual((1, 2), (first.sequence, second.sequence))
        self.assertEqual(10.0, first.captured_at_s)
        self.assertEqual("synthetic_generation", first.timestamp_basis)

    def test_host_timestamps_bracket_capture_read_and_sequence_increases(self) -> None:
        first_image, second_image = object(), object()
        capture = FakeCapture([(True, first_image), (True, second_image)])
        cv2 = fake_cv2(capture)
        events = []
        original_read = capture.read

        def read():
            events.append("read")
            return original_read()

        def clock():
            events.append("clock")
            return float(len(events))

        capture.read = read
        with (
            patch.dict("sys.modules", {"cv2": cv2}),
            patch("robo_control.adapters.time.monotonic", side_effect=clock),
            OpenCVCameraSource(2) as source,
        ):
            first, second = source.read(), source.read()
        self.assertEqual(["clock", "read", "clock"] * 2, events)
        self.assertEqual((1.0, 3.0), (first.captured_at_s, first.received_at_s))
        self.assertEqual((4.0, 6.0), (second.captured_at_s, second.received_at_s))
        self.assertEqual((1, 2), (first.sequence, second.sequence))
        self.assertIs(first_image, first.image)
        self.assertIs(second_image, second.image)
        self.assertEqual("webcam:2", first.source_name)
        self.assertEqual("host_read_start", first.timestamp_basis)
        self.assertFalse(first.is_replay)
        self.assertIsNone(first.media_time_s)
        self.assertEqual([], capture.get_properties)
        self.assertEqual(1, capture.release_count)
        cv2.VideoCapture.assert_called_once_with(2)

    def test_failed_read_closes_source_and_does_not_increment_sequence(self) -> None:
        capture = FakeCapture([(True, object()), (False, None), (True, object())])
        with patch.dict("sys.modules", {"cv2": fake_cv2(capture)}):
            source = OpenCVCameraSource()
            self.assertEqual(1, source.read().sequence)
            self.assertIsNone(source.read())
            self.assertIsNone(source.read())
            source.close()
        self.assertEqual(1, source.sequence)
        self.assertEqual(2, capture.read_count)
        self.assertEqual(1, capture.release_count)

    def test_empty_successful_frame_is_treated_as_failed_read(self) -> None:
        capture = FakeCapture([(True, None)])
        with patch.dict("sys.modules", {"cv2": fake_cv2(capture)}):
            source = OpenCVCameraSource()
            self.assertIsNone(source.read())
        self.assertEqual(1, capture.release_count)
        self.assertEqual(0, source.sequence)

    def test_opencv_read_error_closes_source(self) -> None:
        capture = FakeCapture([CaptureError("disconnected")])
        with patch.dict("sys.modules", {"cv2": fake_cv2(capture)}):
            source = OpenCVCameraSource()
            self.assertIsNone(source.read())
            self.assertIsNone(source.read())
        self.assertEqual(1, capture.read_count)
        self.assertEqual(1, capture.release_count)

    def test_close_is_idempotent_and_closed_source_never_reads(self) -> None:
        capture = FakeCapture([(True, object())])
        with patch.dict("sys.modules", {"cv2": fake_cv2(capture)}):
            source = OpenCVCameraSource()
            source.close()
            source.close()
            self.assertIsNone(source.read())
            with self.assertRaisesRegex(RuntimeError, "closed"):
                source.__enter__()
        self.assertEqual(0, capture.read_count)
        self.assertEqual(1, capture.release_count)

    def test_context_manager_releases_capture_on_consumer_exception(self) -> None:
        capture = FakeCapture()
        with (
            patch.dict("sys.modules", {"cv2": fake_cv2(capture)}),
            self.assertRaisesRegex(ValueError, "consumer failed"),
            OpenCVCameraSource() as source,
        ):
            self.assertIsInstance(source, OpenCVCameraSource)
            raise ValueError("consumer failed")
        self.assertEqual(1, capture.release_count)

    def test_unopened_capture_is_released(self) -> None:
        capture = FakeCapture(opened=False)
        with (
            patch.dict("sys.modules", {"cv2": fake_cv2(capture)}),
            self.assertRaisesRegex(RuntimeError, "could not be opened"),
        ):
            OpenCVCameraSource()
        self.assertEqual(1, capture.release_count)

    def test_open_status_error_releases_capture(self) -> None:
        capture = FakeCapture(opened=CaptureError("backend error"))
        with (
            patch.dict("sys.modules", {"cv2": fake_cv2(capture)}),
            self.assertRaises(CaptureError),
        ):
            OpenCVCameraSource()
        self.assertEqual(1, capture.release_count)

    def test_optional_opencv_dependency_has_actionable_error(self) -> None:
        with (
            patch.dict("sys.modules", {"cv2": None}),
            self.assertRaisesRegex(RuntimeError, r"pip install -e .\[vision\]"),
        ):
            OpenCVCameraSource()

    def test_stream_source_is_distinct_from_webcam(self) -> None:
        capture = FakeCapture([(True, object())])
        with (
            patch.dict("sys.modules", {"cv2": fake_cv2(capture)}),
            OpenCVCameraSource("rtsp://example.invalid/camera") as source,
        ):
            self.assertEqual("opencv:rtsp://example.invalid/camera", source.read().source_name)


class VideoFileSourceTests(unittest.TestCase):
    # An existing source file is sufficient for the fake decoder; tests require
    # neither a video codec nor permission to create temporary directories.
    existing_file = Path(__file__).resolve()

    def test_video_uses_absolute_local_path_and_separate_media_clock(self) -> None:
        image = object()
        capture = FakeCapture([(True, image)], media_positions=[1250.0])
        cv2 = fake_cv2(capture)
        with (
            patch.dict("sys.modules", {"cv2": cv2}),
            patch("robo_control.adapters.time.monotonic", side_effect=[80.0, 80.03]),
            VideoFileSource(self.existing_file) as source,
        ):
            frame = source.read()
        cv2.VideoCapture.assert_called_once_with(str(self.existing_file))
        self.assertIs(image, frame.image)
        self.assertEqual(f"video:{self.existing_file}", frame.source_name)
        self.assertTrue(frame.is_replay)
        self.assertEqual(1.25, frame.media_time_s)
        self.assertEqual((80.0, 80.03), (frame.captured_at_s, frame.received_at_s))
        self.assertEqual([cv2.CAP_PROP_POS_MSEC], capture.get_properties)
        self.assertEqual(1, capture.release_count)

    def test_eof_closes_video_without_looping(self) -> None:
        capture = FakeCapture([(True, object()), (False, None), (True, object())])
        with (
            patch.dict("sys.modules", {"cv2": fake_cv2(capture)}),
            VideoFileSource(self.existing_file) as source,
        ):
            self.assertEqual(1, source.read().sequence)
            self.assertIsNone(source.read())
            self.assertIsNone(source.read())
        self.assertEqual(2, capture.read_count)
        self.assertEqual(1, capture.release_count)

    def test_invalid_media_position_does_not_discard_valid_image(self) -> None:
        for invalid in (float("nan"), float("inf"), -1.0, None, "unknown", CaptureError("not supported")):
            capture = FakeCapture([(True, object())], media_positions=[invalid])
            with (
                self.subTest(invalid=invalid),
                patch.dict("sys.modules", {"cv2": fake_cv2(capture)}),
                VideoFileSource(self.existing_file) as source,
            ):
                frame = source.read()
                self.assertIsNotNone(frame)
                self.assertIsNone(frame.media_time_s)

    def test_missing_file_rejected_before_loading_opencv(self) -> None:
        missing_file = self.existing_file.with_name("not-an-existing-camera-fixture-4785.mp4")
        with (
            patch.dict("sys.modules", {"cv2": None}),
            self.assertRaises(FileNotFoundError),
        ):
            VideoFileSource(missing_file)

    def test_directory_is_not_a_video_file(self) -> None:
        with (
            patch.dict("sys.modules", {"cv2": None}),
            self.assertRaisesRegex(ValueError, "local file"),
        ):
            VideoFileSource(self.existing_file.parent)

    def test_remote_url_is_not_a_local_video_file(self) -> None:
        cv2 = fake_cv2(FakeCapture())
        with (
            patch.dict("sys.modules", {"cv2": cv2}),
            self.assertRaises((OSError, ValueError)),
        ):
            VideoFileSource("https://example.invalid/video.mp4")
        cv2.VideoCapture.assert_not_called()


if __name__ == "__main__":
    unittest.main()
