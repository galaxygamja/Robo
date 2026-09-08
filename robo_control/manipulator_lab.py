"""Four addressed synthetic manipulators; contract checks, not a mission simulator."""
from __future__ import annotations

import argparse
import json

from .fake_manipulator import FakeManipulatorReceiver
from .manipulator_wire import ManipulatorSender
from .wire_codec import encode_frame


def run_demo():
    checks, endpoints = [], {}

    def check(name, passed):
        checks.append({"name": name, "passed": bool(passed)})

    for index, rid in enumerate(("H1", "H2", "B1", "B2")):
        action = "disc_latch_close" if rid == "H1" else "gripper_close"
        sender = ManipulatorSender(rid, session_id="synthetic-lab")
        receiver = FakeManipulatorReceiver(rid, supported_actions={action, "hold"})
        origin = (10000., 2., 90000., 500.)[index]
        endpoints[rid] = (sender, receiver, origin)
        operation = {"command_id": "lab:0:3", "phase": "close_servo", "action": action, "timeout_ms": 1000}

        def exchange(message, t, *, delay=0., receiver=receiver, origin=origin, sender=sender):
            response = receiver.receive_frame(encode_frame(message), origin + t)
            return sender.accept_frame(response, 10. + t + delay)

        hello_ok = exchange(sender.begin(operation, 10.), .01)
        execute = sender.execute(10.02)
        if rid == "H2":
            # Device accepted, but host never sees the ACK until it is too late.
            response = receiver.receive_frame(encode_frame(execute), origin + .03)
            sender.poll(10.32)
            check("H2_lost_ack_no_reauthorization", not sender.accept_frame(response, 10.33)
                  and receiver.accepted_operations == 1)
            check("H2_best_effort_stop", exchange(sender.stop(10.34), .35))
        else:
            check(rid + "_ack_only", hello_ok and exchange(execute, .03)
                  and sender.snapshot()["evidence"] is None)
            receiver.inject_test_sample("lab:0:3", {"servo_closed": True}, origin + .04)
            if rid == "B1":
                check("B1_delayed_sensor_rejected", not exchange(sender.sample(10.05), .06, delay=.19))
                check("B1_best_effort_stop", exchange(sender.stop(10.26), .27))
            else:
                check(rid + "_synthetic_sample", exchange(sender.sample(10.05), .06)
                      and sender.snapshot()["evidence"]["mission_feedback_eligible"] is False)
                if rid == "B2":
                    duplicate = receiver.receive(execute, origin + .07)
                    check("B2_duplicate_never_reexecuted", duplicate["result"] == "rejected"
                          and receiver.accepted_operations == 1)
                check(rid + "_cancel", exchange(sender.stop(10.08, kind="cancel"), .09))
    final = {}
    for rid, (sender, receiver, origin) in endpoints.items():
        sender.poll(11.1)
        final[rid] = receiver.tick(origin + 1.1)
    check("all_logical_actions_cleared", all(r["requested_action"] is None for r in final.values()))
    return {"passed": all(c["passed"] for c in checks), "checks": checks, "receivers": final,
            "transport": "in_memory_framed_bytes", "synthetic": True, "device_io": False,
            "hardware_ready": False, "physical_success": None, "physical_safe_state": "unconfigured",
            "mission_feedback_eligible": False, "mission_completed": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    result = run_demo()
    print(json.dumps(result, ensure_ascii=True, indent=None if args.compact else 2))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
