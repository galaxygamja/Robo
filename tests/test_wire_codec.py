from __future__ import annotations

import json
import unittest

from robo_control.wire_codec import (
    MAX_FRAME_BYTES,
    FrameDecoder,
    WireError,
    decode_frame,
    encode_frame,
)


class WireCodecTests(unittest.TestCase):
    def test_roundtrip_compact_unicode_and_json_types(self):
        message = {"robot_id": "로봇1", "values": [True, False, None, 1, -2, 3.5, {}]}
        frame = encode_frame(message)
        self.assertEqual(message, decode_frame(frame))
        self.assertTrue(frame.endswith(b"\n"))
        self.assertEqual(1, frame.count(b"\n"))
        self.assertIn("로봇1".encode(), frame)
        self.assertNotIn(b": ", frame)

    def test_bool_is_preserved_for_later_semantic_validation(self):
        self.assertIs(True, decode_frame(encode_frame({"version": True}))["version"])

    def test_exact_frame_byte_budget_includes_lf(self):
        message = {"x": "a" * (MAX_FRAME_BYTES - len(b'{"x":""}\n'))}
        frame = encode_frame(message)
        self.assertEqual(MAX_FRAME_BYTES, len(frame))
        self.assertEqual(message, decode_frame(frame))
        with self.assertRaises(WireError):
            encode_frame({"x": message["x"] + "a"})
        with self.assertRaises(WireError):
            decode_frame(frame[:-1] + b" \n")

    def test_byte_budget_is_not_unicode_character_count(self):
        with self.assertRaises(WireError):
            encode_frame({"x": "가" * 1400})
        self.assertEqual({"x": "가" * 1000}, decode_frame(encode_frame({"x": "가" * 1000})))

    def test_crlf_is_supported_and_counts_both_terminator_bytes(self):
        self.assertEqual({"x": 1}, decode_frame(b'{"x":1}\r\n'))
        maximum = encode_frame({"x": "a" * (MAX_FRAME_BYTES - len(b'{"x":""}\n'))})
        with self.assertRaises(WireError):
            decode_frame(maximum[:-1] + b"\r\n")

    def test_literal_line_breaks_and_multiple_frames_are_rejected(self):
        for frame in (b'{\n"x":1}\n', b'{}\n{}\n', b'{}\r \n', b'{\r"x":1}\n'):
            with self.subTest(frame=frame), self.assertRaises(WireError):
                decode_frame(frame)
        self.assertEqual({"x": "\n\r"}, decode_frame(b'{"x":"\\n\\r"}\n'))

    def test_missing_terminator_blank_and_trailing_garbage_are_rejected(self):
        for frame in (b"", b"{}", b"\n", b" \n", b"{}{}\n", b"{} garbage\n"):
            with self.subTest(frame=frame), self.assertRaises(WireError):
                decode_frame(frame)

    def test_invalid_utf8_bom_and_unpaired_unicode_surrogates_are_rejected(self):
        for frame in (b'{"x":"\xff"}\n', b'\xef\xbb\xbf{}\n', b'{"x":"\\ud800"}\n'):
            with self.subTest(frame=frame), self.assertRaises(WireError):
                decode_frame(frame)
        for message in ({"x": "\ud800"}, {"\ud800": "x"}):
            with self.subTest(message=repr(message)), self.assertRaises(WireError):
                encode_frame(message)
        self.assertEqual({"x": "😀"}, decode_frame(b'{"x":"\\ud83d\\ude00"}\n'))

    def test_duplicate_keys_are_rejected_at_every_depth(self):
        frames = (
            b'{"x":1,"x":2}\n',
            b'{"x":[{"a":1,"a":2}]}\n',
            b'{"x":1,"\\u0078":2}\n',
        )
        for frame in frames:
            with self.subTest(frame=frame), self.assertRaises(WireError):
                decode_frame(frame)

    def test_nonfinite_numbers_are_rejected_in_both_directions(self):
        for number in (b"NaN", b"Infinity", b"-Infinity", b"1e400"):
            with self.subTest(number=number), self.assertRaises(WireError):
                decode_frame(b'{"x":[' + number + b"]}\n")
        for number in (float("nan"), float("inf"), -float("inf")):
            with self.subTest(number=number), self.assertRaises(WireError):
                encode_frame({"x": [number]})

    def test_integers_must_fit_signed_64_bits(self):
        for number in (-(2**63), 2**63 - 1):
            self.assertEqual({"x": number}, decode_frame(encode_frame({"x": number})))
        for number in (-(2**63) - 1, 2**63):
            with self.subTest(number=number):
                with self.assertRaises(WireError):
                    encode_frame({"x": number})
                with self.assertRaises(WireError):
                    decode_frame(json.dumps({"x": number}).encode() + b"\n")
        with self.assertRaises(WireError):
            decode_frame(b'{"x":' + b"9" * 3000 + b"}\n")

    def test_nested_containers_are_limited_to_16_including_root(self):
        message = {"x": 1}
        for _ in range(15):
            message = {"nested": message}
        self.assertEqual(message, decode_frame(encode_frame(message)))
        too_deep = {"nested": message}
        with self.assertRaises(WireError):
            encode_frame(too_deep)
        with self.assertRaises(WireError):
            decode_frame(json.dumps(too_deep).encode() + b"\n")

    def test_pathological_depth_and_cycles_raise_wire_error(self):
        with self.assertRaises(WireError):
            decode_frame(b'{"x":' + b"[" * 1500 + b"0" + b"]" * 1500 + b"}\n")
        message = {}
        message["cycle"] = message
        with self.assertRaises(WireError):
            encode_frame(message)

    def test_nonobject_roots_and_non_json_python_values_are_rejected(self):
        for value in ([], "text", 1, True, None):
            with self.subTest(value=value):
                with self.assertRaises(WireError):
                    encode_frame(value)
                with self.assertRaises(WireError):
                    decode_frame(json.dumps(value).encode() + b"\n")
        for value in ({1: "key"}, {"x": (1, 2)}, {"x": b"text"}, {"x": {1, 2}}):
            with self.subTest(value=value), self.assertRaises(WireError):
                encode_frame(value)

    def test_frame_and_chunk_require_bytes(self):
        for frame in ("{}\n", bytearray(b"{}\n"), memoryview(b"{}\n"), None):
            with self.subTest(frame=frame), self.assertRaises(WireError):
                decode_frame(frame)

    def test_encode_does_not_mutate_and_decode_results_are_independent(self):
        message = {"nested": [{"x": 1}]}
        frame = encode_frame(message)
        first, second = decode_frame(frame), decode_frame(frame)
        first["nested"][0]["x"] = 9
        self.assertEqual(1, message["nested"][0]["x"])
        self.assertEqual(1, second["nested"][0]["x"])
        message["nested"].clear()
        self.assertEqual(1, decode_frame(frame)["nested"][0]["x"])


class FrameDecoderTests(unittest.TestCase):
    def test_every_split_boundary_including_multibyte_unicode(self):
        message = {"robot_id": "로봇1", "sequence": 3}
        frame = encode_frame(message)
        for split in range(len(frame) + 1):
            with self.subTest(split=split):
                decoder = FrameDecoder()
                results = decoder.feed(frame[:split]) + decoder.feed(frame[split:])
                self.assertEqual([message], results)
                self.assertEqual(0, decoder.buffered_bytes)
                decoder.finish()

    def test_byte_at_a_time_and_crlf(self):
        decoder = FrameDecoder()
        results = []
        for byte in b'{"x":1}\r\n':
            results.extend(decoder.feed(bytes([byte])))
        self.assertEqual([{"x": 1}], results)

    def test_many_coalesced_frames_are_not_a_single_oversize_frame(self):
        decoder = FrameDecoder()
        frames = b"".join(encode_frame({"sequence": n}) for n in range(5000))
        self.assertGreater(len(frames), MAX_FRAME_BYTES)
        self.assertEqual([{"sequence": n} for n in range(5000)], decoder.feed(frames))
        self.assertEqual(0, decoder.buffered_bytes)

    def test_coalesced_frames_keep_only_last_partial(self):
        decoder = FrameDecoder()
        self.assertEqual([{"a": 1}, {"a": 2}], decoder.feed(b'{"a":1}\n{"a":2}\n{"a":'))
        self.assertEqual(len(b'{"a":'), decoder.buffered_bytes)
        self.assertEqual([{"a": 3}], decoder.feed(b"3}\n"))

    def test_largest_legal_frame_can_arrive_in_two_chunks(self):
        decoder = FrameDecoder()
        message = {"x": "a" * (MAX_FRAME_BYTES - len(b'{"x":""}\n'))}
        frame = encode_frame(message)
        self.assertEqual([], decoder.feed(frame[:-1]))
        self.assertEqual(MAX_FRAME_BYTES - 1, decoder.buffered_bytes)
        self.assertEqual([message], decoder.feed(frame[-1:]))

    def test_oversize_unterminated_chunk_faults_and_discards_buffer(self):
        decoder = FrameDecoder()
        with self.assertRaises(WireError):
            decoder.feed(b"x" * (MAX_FRAME_BYTES * 100))
        self.assertTrue(decoder.closed)
        self.assertTrue(decoder.faulted)
        self.assertEqual(0, decoder.buffered_bytes)
        with self.assertRaises(WireError):
            decoder.feed(b"{}\n")

    def test_frame_exceeding_limit_across_chunks_faults_before_later_lf(self):
        decoder = FrameDecoder()
        self.assertEqual([], decoder.feed(b"x" * (MAX_FRAME_BYTES - 1)))
        with self.assertRaises(WireError):
            decoder.feed(b"x\n{}\n")
        self.assertEqual(0, decoder.buffered_bytes)

    def test_any_invalid_frame_discards_entire_batch_without_resynchronizing(self):
        decoder = FrameDecoder()
        received = []
        with self.assertRaises(WireError):
            received.extend(decoder.feed(b'{"sequence":1}\n{"x":NaN}\n{"sequence":2}\n'))
        self.assertEqual([], received)
        self.assertTrue(decoder.faulted)
        with self.assertRaises(WireError):
            decoder.feed(b'{"sequence":3}\n')

    def test_empty_chunk_is_no_data_and_does_not_refresh_partial(self):
        decoder = FrameDecoder()
        self.assertEqual([], decoder.feed(b""))
        self.assertEqual([], decoder.feed(b"{"))
        self.assertEqual([], decoder.feed(b""))
        self.assertEqual(1, decoder.buffered_bytes)
        self.assertFalse(decoder.closed)

    def test_eof_closes_clean_decoder_and_is_idempotent(self):
        decoder = FrameDecoder()
        decoder.feed(b"{}\n")
        self.assertIsNone(decoder.finish())
        self.assertIsNone(decoder.finish())
        self.assertTrue(decoder.closed)
        self.assertFalse(decoder.faulted)
        with self.assertRaises(WireError):
            decoder.feed(b"{}\n")

    def test_truncated_eof_is_a_latched_fault_even_after_prior_good_frames(self):
        decoder = FrameDecoder()
        self.assertEqual([{}], decoder.feed(b"{}\n{"))
        with self.assertRaises(WireError):
            decoder.finish()
        self.assertEqual(0, decoder.buffered_bytes)
        self.assertTrue(decoder.faulted)
        with self.assertRaises(WireError):
            decoder.finish()

    def test_reset_discards_partial_and_recovers_closed_or_faulted_decoder(self):
        decoder = FrameDecoder()
        decoder.feed(b'{"old":')
        decoder.reset()
        self.assertEqual([{"new": 1}], decoder.feed(b'{"new":1}\n'))
        decoder.finish()
        decoder.reset()
        self.assertEqual([{}], decoder.feed(b"{}\n"))
        with self.assertRaises(WireError):
            decoder.feed(b"bad\n")
        decoder.reset()
        self.assertFalse(decoder.closed)
        self.assertFalse(decoder.faulted)
        self.assertEqual([{}], decoder.feed(b"{}\n"))

    def test_nonbyte_chunk_faults(self):
        for chunk in ("{}\n", bytearray(b"{}\n"), None):
            decoder = FrameDecoder()
            with self.subTest(chunk=chunk), self.assertRaises(WireError):
                decoder.feed(chunk)
            self.assertTrue(decoder.faulted)

    def test_returned_nested_messages_share_no_state(self):
        decoder = FrameDecoder()
        first, second = decoder.feed(b'{"x":[1]}\n{"x":[1]}\n')
        first["x"].append(2)
        self.assertEqual({"x": [1]}, second)
        self.assertEqual([{"x": [1]}], decoder.feed(b'{"x":[1]}\n'))


if __name__ == "__main__":
    unittest.main()
