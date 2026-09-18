"""Live measured-camera bridge to explicitly enabled differential-drive hardware.

Without --enable-hardware this runs camera validation and command calculation
only: no UDP socket, handshake, or physical command is created. Video/replay is
intentionally absent. A physical fault closes the entire session; restart is an
operator action, never an automatic reconnect or replay of buffered commands.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import multiprocessing
import os
import sys
import time
from copy import deepcopy
from dataclasses import asdict, replace
from pathlib import Path

from .fleet import validate_tag_registry
from .control_loop import ControlLimits
from .mission_bindings import load_binding_plan, load_review_json, profile_identity
from .runtime import camera_worker, load_goals
from .runtime_io import AsyncJsonlReport, LatestRecordMailbox
from .runtime_session import LiveControlSession
from .vision.calibration import FieldCalibration
from .vision.colors import ColorDetector
from .vision.tags import TagDetectorConfig, default_tag_config_path


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def calibrated_limits(config):
    """Leave half the slowest measured wheel range for translation and turning.

    With |v| <= S/2 and |omega| <= S/max_track, every wheel's
    |v +/- omega*track/2| is <= S. This prevents clipping an unachievable
    controller request; the transport still checks the individual curves.
    """
    config.require_ready()
    speed = min(getattr(robot, curve)[-1][1] for robot in config.robots.values()
                for curve in ("left_forward", "left_reverse", "right_forward", "right_reverse"))
    track = max(robot.track_width_mm for robot in config.robots.values())
    return replace(ControlLimits(), max_speed_mm_s=min(180., speed / 2.),
                   max_turn_rad_s=min(1.5, speed / track), command_ttl_s=.25)


class HardwareRuntime:
    """Supervise a LiveControlSession and one nonblocking fleet transport.

    start() authorizes one connection attempt. Three verified observations
    precede that attempt. ACK permits are read before advance() creates a new
    packet, and packets are never saved for transmission on a later tick.
    Servo ACKs report a requested pulse, not a measured mechanical position.
    """

    def __init__(self, session, config, transport=None, *, enabled=False, clock=time.monotonic):
        if type(enabled) is not bool:
            raise ValueError("Hardware enable flag must be Boolean")
        if session.is_replay or not session.source_name.startswith("webcam:"):
            raise ValueError("Physical bridge accepts a live camera only, never replay")
        if session.controller.drive_model != "differential_body" or session.wire is not None:
            raise ValueError("Hardware bridge requires differential body commands without fake wire")
        if set(session.controller.ids) != set(config.robots):
            raise ValueError("Camera, controller and hardware robot registries must match exactly")
        if any(session.controller.roles[rid] != profile.role for rid, profile in config.robots.items()):
            raise ValueError("Controller and hardware roles must match for every robot")
        if enabled:
            config.require_ready()
            if transport is None:
                raise ValueError("Explicit hardware mode requires a transport")
            for rid, profile in config.robots.items():
                if session.controller.radii_mm[rid] < profile.envelope_radius_mm:
                    raise ValueError(f"Controller collision radius is smaller than physical envelope: {rid}")
        elif transport is not None:
            raise ValueError("Dry mode must not create a hardware transport")
        self.session, self.config, self.transport = session, config, transport
        self.enabled, self.clock = enabled, clock
        self.authorized = False
        self.connected = False
        self.closed_reason = None
        self.sent_packets = 0
        self.last_sent_sequence = None
        self.servo_us = {rid: (0, 0) for rid in config.robots}

    def start(self, *, operator_confirmed=False):
        if self.closed_reason is not None or self.authorized:
            raise ValueError("A runtime can be started once; faults require a new session")
        if self.enabled and operator_confirmed is not True:
            raise ValueError("Physical hardware start needs explicit operator authorization")
        self.authorized = True

    def _event(self, event, *, sent=False):
        event = deepcopy(event)
        snapshot = self.transport.snapshot() if self.transport else None
        event.update(output_mode="physical_udp" if self.enabled else "dry_run_commands",
                     transport_mode="physical_udp" if self.enabled else "mock",
                     hardware=snapshot, device_io=bool(self.connected),
                     hardware_ready=bool(self.transport and self.transport.ready and not self.closed_reason),
                     motion_permitted=bool(sent), hardware_command_sent=bool(sent),
                     sent_packets=self.sent_packets, hardware_closed_reason=self.closed_reason,
                     physical_mission_verified=False,
                     feedback_policy="measured_disc_sensor_only; servo/grip/hopper confirmation unavailable")
        return event

    def _feedback(self, now_s):
        """Return only fresh, calibrated actual optical telemetry for H1.

        No servo completion, object release, hopper count or gripper presence
        is guessed from an ACK, elapsed time, or the requested servo pulse.
        """
        mission = self.session.mission
        if not self.transport or not mission or not mission.active:
            return None
        rid = mission.active.task.robot_id
        profile = self.config.robots[rid]
        if profile.role != "hamster" or not profile.sensor_verified:
            return None
        snapshot = self.transport.snapshot()
        robots = snapshot.get("robots", {})
        state = robots.get(rid) if isinstance(robots, dict) else next(
            (r for r in robots if r.get("robot_id") == rid), None)
        if not isinstance(state, dict):
            return None
        # The request send time is the earliest possible sample time. Using
        # receipt time would rejuvenate an old sensor sample delayed in Wi-Fi.
        telemetry = state.get("telemetry", {})
        stamp = telemetry.get("request_sent_at_s") if isinstance(telemetry, dict) else None
        present = telemetry.get("disc_present") if isinstance(telemetry, dict) else None
        if (not _finite(stamp) or not 0 <= now_s - stamp < .2 or type(present) is not bool):
            return None
        return {"session_id": mission.session_id, "command_id": mission.command_id,
                "robot_id": rid, "synthetic": False, "observed_at_s": stamp,
                "signals": {"optical_present": present, "optical_clear": not present}}

    def _servo_targets(self, event):
        mission = event.get("mission")
        intent = mission.get("manipulator_intent") if mission else None
        if intent and intent.get("servo_intent") != "hold":
            rid, name = intent["robot_id"], intent["servo_intent"]
            preset = self.config.robots[rid].servo_presets.get(name)
            if preset is None:
                raise ValueError(f"unsupported_servo_intent:{rid}:{name}")
            self.servo_us[rid] = tuple(preset)
        return dict(self.servo_us)

    def advance(self, record, now_s):
        started = self.clock()
        if not _finite(now_s) or now_s < 0 or (self.session.now is not None and now_s < self.session.now):
            self.close(self.session.now or 0.0, "invalid_runtime_clock")
            raise ValueError("Runtime clock must be finite and monotonic")
        if self.closed_reason is not None:
            return self._event(self.session.advance(None, now_s, command_permitted=False))
        if not self.authorized:
            raise ValueError("Call start() before advancing the runtime")
        if (self.session.now is not None and now_s - self.session.now > self.session.max_tick_gap_s + 1e-9):
            return self.close(now_s, "supervisor_deadline_missed")
        if (self.connected and self.session.last_capture_s is not None
                and now_s >= self.session.last_capture_s + self.session.controller.limits.max_pose_age_s):
            return self.close(now_s, "observation_watchdog")
        try:
            if self.connected:
                self.transport.poll(now_s)
                if self.transport.fault:
                    return self.close(now_s, "hardware_fault:" + str(self.transport.fault))
            permit = not self.enabled or bool(self.connected and self.transport.ready)
            event = self.session.advance(record, now_s, feedback=self._feedback(now_s),
                                         command_permitted=permit)
            elapsed = max(0.0, self.clock() - started)
            finished = now_s + elapsed
            if elapsed > self.session.max_tick_gap_s:
                return self.close(finished, "supervisor_deadline_missed")
            if self.session.closed_reason:
                return self.close(finished, self.session.closed_reason)
            observation = event.get("observation")
            good_frame = (isinstance(observation, dict)
                          and observation.get("observation_usable") is True
                          and _finite(observation.get("captured_at_s"))
                          and 0 <= finished - observation["captured_at_s"] < .2
                          and self.session.fresh_streak > 0)
            if self.connected and record is not None and not good_frame:
                return self.close(finished, "invalid_observation:" + str(event["status"]))
            if (self.enabled and not self.connected and good_frame
                    and self.session.fresh_streak >= self.session.recovery_frames):
                self.connected = True
                self.transport.connect(finished)
                if self.transport.fault:
                    return self.close(finished, "hardware_fault:" + str(self.transport.fault))
                event["status"] = "hardware_connecting"
                return self._event(event)
            # ACKs only grant a future freshly calculated packet. A None poll
            # must not replay the most recent command or extend its TTL.
            if not (self.enabled and permit and good_frame and event.get("command") is not None):
                return self._event(event)
            packet = event["command"]
            if packet.get("stop_reason") is not None:
                return self.close(finished, "controller_stop:" + str(packet["stop_reason"]))
            try:
                servos = self._servo_targets(event)
            except ValueError as exc:
                return self.close(finished, str(exc))
            if not self.transport.send(packet, finished, servo_us=servos):
                return self.close(finished, "hardware_dispatch_rejected:" + str(self.transport.fault or "no_permit"))
            self.sent_packets += 1
            self.last_sent_sequence = packet.get("sequence")
            return self._event(event, sent=True)
        except Exception:
            self.close(max(now_s, self.session.now or now_s), "hardware_runtime_failed")
            raise

    def close(self, now_s, reason="operator_stop"):
        if self.closed_reason is None:
            self.closed_reason = reason
            # Physical stop must precede tracker diagnostics and report I/O.
            if self.transport:
                try:
                    self.transport.stop(reason, emergency=reason not in {"operator_stop", "duration_elapsed", "mission_completed"})
                finally:
                    self.transport.close()
        return self._event(self.session.close(max(now_s, self.session.now or 0.0), self.closed_reason))


def supervise(runtime, worker, mailbox, stop, worker_status, report, *, duration_s=120.,
              tick_hz=50., startup_timeout_s=10.):
    """Camera decoding lives in a separate process; hardware ticks never block."""
    started = next_tick = time.monotonic()
    revision = frames = usable_frames = 0
    reason, error = "duration_elapsed", None
    try:
        while time.monotonic() < started + duration_s:
            now = time.monotonic()
            if report.error is not None:
                reason = "report_failed"
                break
            if now >= started + startup_timeout_s and frames == 0:
                reason = "camera_startup_timeout"
                break
            value = mailbox.poll(revision)
            record = None
            if value is not None:
                revision, record = value
                frames += 1
                if record.get("status") == "source_closed":
                    reason = str(record.get("reason", "source_closed"))
                    break
            if worker_status.value in (2, 3, 4, 5) or not worker.is_alive():
                reason = "camera_worker_failed"
                break
            event = runtime.advance(record, now)
            if event.get("observation") and event["observation"].get("observation_usable") is True:
                usable_frames += 1
            if not report.submit(event):
                reason = "report_failed"
                break
            if runtime.closed_reason:
                reason = runtime.closed_reason
                break
            next_tick = max(next_tick + 1.0 / tick_hz, time.monotonic())
            stop.wait(max(0.0, next_tick - time.monotonic()))
            if stop.is_set():
                reason = "operator_stop"
                break
    except KeyboardInterrupt:
        reason = "operator_stop"
    except Exception as exc:
        reason, error = "hardware_runtime_failed", type(exc).__name__
    finally:
        final = runtime.close(time.monotonic(), reason)
        stop.set()
        report.submit(final)
        worker.join(timeout=.5)
        if worker.is_alive():
            worker.terminate()
            worker.join(timeout=.5)
        if worker.is_alive():
            worker.kill()
            worker.join(timeout=.5)
        complete = report.finish()
    return {"status": reason, "error_type": error, "frames": frames, "usable_frames": usable_frames,
            "sent_packets": runtime.sent_packets, "report_complete": complete,
            "worker_stopped": not worker.is_alive(), "output_mode": final["output_mode"],
            "physical_mission_verified": False, "final": final}


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware-config", type=Path, required=True)
    parser.add_argument("--enable-hardware", action="store_true", help="explicitly authorize physical UDP output")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--tags", type=Path, default=default_tag_config_path())
    parser.add_argument("--fleet", type=Path, required=True)
    run = parser.add_mutually_exclusive_group()
    run.add_argument("--mission", type=Path)
    run.add_argument("--goals", type=Path)
    parser.add_argument("--colors", type=Path)
    parser.add_argument("--bindings", type=Path)
    parser.add_argument("--object-obstacles", type=Path)
    parser.add_argument("--report", type=Path, required=True, help="new JSONL path; never overwrite")
    parser.add_argument("--duration-s", type=float, default=120.)
    parser.add_argument("--tick-hz", type=float, default=50.)
    return parser


def main(argv=None):
    from .hardware import HardwareConfig, UdpFleetTransport

    args = _parser().parse_args(argv)
    runtime = transport = report = worker = stop = None
    try:
        if (args.camera < 0 or not _finite(args.duration_s) or not 0 < args.duration_s <= 120
                or not _finite(args.tick_hz) or not 20 <= args.tick_hz <= 100):
            raise ValueError("Camera >=0, duration (0,120] seconds, tick rate [20,100] required")
        if (args.bindings or args.object_obstacles) and not (args.mission and args.colors):
            raise ValueError("Bindings/obstacles require both a mission and colour profiles")
        inputs = [args.hardware_config, args.calibration, args.tags, args.fleet, args.mission,
                  args.goals, args.colors, args.bindings, args.object_obstacles]
        if args.report.resolve() in {p.resolve() for p in inputs if p is not None}:
            raise ValueError("Report path must differ from every input")
        config = HardwareConfig.load(args.hardware_config)
        calibration = FieldCalibration.load(args.calibration)
        tags = TagDetectorConfig.load(args.tags)
        fleet = json.loads(args.fleet.read_text(encoding="utf-8-sig"))
        roles = validate_tag_registry(fleet, tags.tag_to_robot)
        colors = json.loads(args.colors.read_text(encoding="utf-8-sig")) if args.colors else None
        if colors:
            ColorDetector(calibration, colors)
        mission = load_review_json(args.mission, name="Mission") if args.mission else None
        bindings = load_binding_plan(args.bindings) if args.bindings else None
        obstacles = load_review_json(args.object_obstacles, name="Object obstacle") if args.object_obstacles else None
        goals, radii = load_goals(args.goals, roles, calibration)
        limits = calibrated_limits(config) if args.enable_hardware else replace(ControlLimits(), command_ttl_s=.25)
        hardware_profile = {"schema_version": 1, "network_confirmed": config.network_confirmed,
            "robots": {rid: {key: value for key, value in asdict(profile).items() if key != "robot_id"}
                       for rid, profile in config.robots.items()}}
        options = {"calibration": calibration.as_dict(), "tags": tags.as_dict(), "fleet": fleet,
                   "colors": colors, "mission": mission, "bindings": bindings,
                   "object_obstacles": obstacles, "camera": args.camera, "video": None,
                   "control_limits": vars(limits), "hardware_profile": hardware_profile}
        # Worker constructor expects config fields, not display-only metadata.
        options["tags"] = {"dictionary_name": tags.dictionary_name,
            "tag_to_robot": dict(tags.tag_to_robot),
            "heading_offsets_rad": dict(tags.heading_offsets_rad),
            "robot_center_from_tag_mm": dict(tags.robot_center_from_tag_mm),
            "allowed_margin_mm": tags.allowed_margin_mm, "tag_size_mm": tags.tag_size_mm,
            "tag_size_tolerance_fraction": tags.tag_size_tolerance_fraction,
            "hardware_verified": tags.hardware_verified}
        config_id = hashlib.sha256(json.dumps(options, sort_keys=True, allow_nan=False).encode()).hexdigest()
        options["configuration_id"] = config_id
        session = LiveControlSession(roles=roles, field_size_mm=calibration.field_size_mm,
            source_name=f"webcam:{args.camera}", goals=goals, radii_mm=radii,
            configuration_id=config_id, track_objects=colors is not None, mission_plan=mission,
            fleet=fleet, binding_plan=bindings, obstacle_plan=obstacles, drive_model="differential_body", limits=limits,
            observation_profile_id=profile_identity(calibration.as_dict(), tags.as_dict(), colors))
        if session.bindings:
            classes = {(p["kind"], p["color"]) for p in colors["profiles"]}
            if any((b["kind"], b["colour"]) not in classes for b in session.bindings.plan["bindings"]):
                raise ValueError("Every bound object needs an enabled colour detector profile")
        if session.object_obstacles:
            classes = {(p["kind"], p["color"]) for p in colors["profiles"]}
            footprints = {(p["kind"], p["colour"]) for p in session.object_obstacles.plan["footprints"]}
            if not classes <= footprints:
                raise ValueError("Every detector class needs a reviewed object obstacle footprint")
        if args.enable_hardware:
            config.require_ready()
            if (tags.hardware_verified is not True or tags.tag_size_mm is None
                    or set(tags.heading_offsets_rad) != set(roles)
                    or set(tags.robot_center_from_tag_mm) != set(roles)):
                raise ValueError("Physical mode requires a measured tag size, each robot's heading/axle-centre offsets, and hardware_verified=true")
            token = os.environ.get("ROBO_HW_TOKEN", "")
            transport = UdpFleetTransport(config, token)
        runtime = HardwareRuntime(session, config, transport, enabled=args.enable_hardware)
        context = multiprocessing.get_context("spawn")
        mailbox, stop = LatestRecordMailbox(context), context.Event()
        worker_status = context.RawValue("i", 0)
        report = AsyncJsonlReport(args.report)
        report.submit({"schema_version": 1, "event": "hardware_session_started",
                       "configuration_id": config_id, "roles": roles,
                       "enabled": args.enable_hardware, "source": session.source_name,
                       "physical_mission_verified": False, "drive_model": "differential_body",
                       "control_limits": vars(limits), "hardware_profile": hardware_profile})
        runtime.start(operator_confirmed=args.enable_hardware)
        worker = context.Process(target=camera_worker, args=(options, mailbox, stop, worker_status),
                                 name="robo-hardware-camera", daemon=True)
        worker.start()
        summary = supervise(runtime, worker, mailbox, stop, worker_status, report,
                            duration_s=args.duration_s, tick_hz=args.tick_hz)
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
        return 0 if (summary["status"] in {"duration_elapsed", "operator_stop", "mission_completed"}
                     and summary["usable_frames"] > 0 and summary["report_complete"] and summary["worker_stopped"]) else 1
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        # Do not include arbitrary transport exceptions or environment values
        # in output; configuration messages never contain the authentication key.
        message = str(exc)
        secret = os.environ.get("ROBO_HW_TOKEN", "")
        if secret:
            message = message.replace(secret, "[redacted]")
        print(f"hardware_runtime: {type(exc).__name__}: {message}", file=sys.stderr)
        return 2
    finally:
        if runtime and runtime.closed_reason is None:
            runtime.close(time.monotonic(), "runtime_exit")
        elif transport:
            transport.close()
        if stop:
            stop.set()
        if worker and worker.is_alive():
            worker.terminate()
            worker.join(timeout=.5)
        if report:
            report.finish()


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
