# robo-hw v1 transport / firmware 2.0.0 actuator profiles

One UTF-8 JSON object per UDP datagram, port **4210**, at most **1024 bytes**. Send one request at a time and await its response. Do not retry motion with an old permit/sequence. An ambiguous timeout requires stopping and a fresh handshake. Keep the same UDP source IP and port for a session. This is not the simulator's legacy `robo-wire` transport.

## Identity and actuator metadata

| Sketch profile | Display ID | Wire `robot_id` | `robot_role` | `drive_type` |
|---:|---|---|---|---|
| 1 | H1 | H1 | hamster | continuous_servo |
| 2 | B1 | B1 | beaver | dc_hbridge |
| 3 | B2 | B2 | beaver | dc_hbridge |
| 4 | B3 | H2 | beaver | dc_hbridge |

Wire H2 preserves the existing third Beaver's AprilTag/server identity. It is not a second Hamster. This release changes the Arduino firmware only. The existing Python server and its **schema_version 1** hardware profile format are unchanged. Do not reuse old `motion_calibrated:true` blindly: H1's old DC motor curves do not describe its continuous-rotation servos. Create a separate local profile and remeasure before enabling physical travel.

Transport `version` remains 1, while responses identify `firmware_version:"2.0.0"`. Role, display ID, drive type, maximum and drive pulses are added as telemetry. The unchanged server does **not** automatically enforce these new capability fields. Before commissioning, the operator must match the board's startup log, sketch profile and physical robot; successful JSON parsing alone does not prove that the correct actuator profile was uploaded.

## Requests

All requests have exactly these base fields:

```json
{"protocol":"robo-hw","version":1,"robot_id":"H1","token":"EXAMPLE_TOKEN_NOT_FOR_REAL_USE_123456","request_id":"req_1","type":"hello"}
```

`request_id` is 1–64 ASCII letters/digits/underscore/hyphen. Token is 24–64 characters in the same alphabet, matching the local sketch settings or ignored `include/secrets.h`. Never commit a real token. JSON bools and floats cannot stand in for integers. Duplicate root keys, unknown fields, embedded NULs, trailing JSON/garbage, invalid arrays and packets over the limit are rejected. An invalid packet from the established endpoint disarms the robot; incorrect tokens or another robot's ID do not acquire authority. Authenticated `stop`/`estop` for the correct robot are honored even with an old optional sequence or malformed nonessential fields.

| Type | Additional fields | Effect |
|---|---|---|
| hello | none | Disable outputs, disarm, establish endpoint, new random permit |
| arm | boot_id, permit, seq | Consume permit younger than 250 ms; arm without output; initial lease 250 ms; new permit |
| command | boot_id, permit, seq, ttl_ms, left_pwm, right_pwm, servo_us | Consume fresh permit, apply absolute setpoints, new permit |
| status | none | Read only; no permit renewal or lease extension |
| stop | optional seq | Disable all outputs, disarm, invalidate permit |
| estop | optional seq | Disable all outputs, latch estop until reboot |

`boot_id` and `permit` are 32 lowercase hex characters generated on device. Boot ID changes on reboot; permit changes after hello, arm and accepted command. Invalidated permits are the empty string. `seq` is an integer in 1..2147483647, strictly increasing **across all sessions in one boot**; choose `response.seq + 1`. Hello does not reset sequence. Sequence exhaustion requires reboot.

`ttl_ms` is 1..250. Deadline is **permit issue time + ttl_ms**, using the device's wrap-safe monotonic clock, never receipt time + TTL. The previous output lease is checked before handling a new message. Expired output cannot be rearmed by queued commands. A permit older than the requested TTL also stops the robot.

## Logical drive values and actual pulse values

`left_pwm/right_pwm` are integers in -96..96 with the default build. Positive means calibrated logical forward, negative reverse. These fields retain their historical names for transport compatibility but are **not measured velocity and not uniformly DC duty**:

- `dc_hbridge`: after configured wheel polarity, magnitude is 8-bit DC PWM duty; zero disables both bridge inputs (coast).
- `continuous_servo`: after configured polarity, zero maps to that wheel's `*_NEUTRAL_US`; nonzero values map linearly toward `*_MIN_US` or `*_MAX_US`, reaching that endpoint at the configured maximum magnitude. The defaults 1380/1500/1620 µs are unmeasured trial settings, not product-specific calibration.

`servo_us` is exactly two integers for **tool** servos, not drive servos. Each is 0 (no pulse) or within that channel's configured absolute pulse bounds (outer allowed envelope 900..2100 µs). H1's second tool value must always be 0. The H1 gate is tool 0; Beaver front gripper is tool 0 and rear drop gate tool 1. Setpoints are absolute/idempotent; there is no blind "drop one more kit" operation.

Example stationary command, after obtaining the current boot ID/permit:

```json
{"protocol":"robo-hw","version":1,"robot_id":"H1","token":"EXAMPLE_TOKEN_NOT_FOR_REAL_USE_123456","request_id":"req_3","type":"command","boot_id":"00112233445566778899aabbccddeeff","permit":"ffeeddccbbaa99887766554433221100","seq":2,"ttl_ms":200,"left_pwm":0,"right_pwm":0,"servo_us":[0,0]}
```

Even this nominal zero command can rotate an incorrectly calibrated H1 servo. Run initial tests with drive wheels raised and a physical power disconnect available.

## Responses

Authenticated replies include the original state fields and mandatory actuator metadata. The following illustrates an armed, hardware-enabled H1 using the unmeasured default neutral values; it is **not real telemetry or a recommended calibration**:

```json
{"protocol":"robo-hw","version":1,"type":"response","robot_id":"H1","request_id":"req_3","boot_id":"00112233445566778899aabbccddeeff","permit":"112233445566778899aabbccddeeff00","state":"armed","accepted":true,"reason":"ok","seq":2,"lease_remaining_ms":190,"left_pwm":0,"right_pwm":0,"servo_us":[0,0],"disc_present":false,"hardware_enabled":true,"calibration_confirmed":true,"uptime_ms":1000,"firmware_version":"2.0.0","drive_type":"continuous_servo","robot_role":"hamster","display_id":"H1","max_pwm":96,"drive_servo_us":[1500,1500],"capabilities":{"tool_servos":1,"disc_sensor":true,"strafe":false}}
```

`state` is disarmed/armed/estop. Logical drive/tool values report applied requested outputs before polarity inversion, **not encoder speed, servo angle or physical arrival**. `drive_servo_us` reports the two issued drive-servo pulse widths for H1, or `[0,0]` for DC profiles. H1 pulses are also `[0,0]` when disabled/disarmed. Hardware is enabled only if both compile-time `HARDWARE_OUTPUT_ENABLED` and `ACTUATOR_CALIBRATION_CONFIRMED` are 1. The acknowledgment flag does not automatically measure or certify calibration.

`calibration_confirmed` reports the compile-time human acknowledgment flag, not a measured result. `capabilities.tool_servos` is 1 for H1 and 2 for Beavers; `disc_sensor` is true only for H1. `strafe` is false for all profiles: these robots use two-wheel differential drive, not sideways translation. These additional fields are not enforced by the unchanged Python server.

`disc_present` is a digital sensor reading for H1 and null for robots without a sensor; it does not prove transport success. `lease_remaining_ms` is the fresh permit lifetime after hello (initially 250 ms); while armed it is the smaller of permit lifetime and current output lease lifetime. With no valid permit it is 0. A status reply exposes the current permit without changing its age.

Boot/arm/disarm/stop/estop disable pulses. An active H1 zero command emits calibrated neutral; disarm emits **no pulse**, not neutral. The stop behavior of the physical servo and load holding must be tested. Software output inhibition is not a mechanical brake or a safety-rated emergency stop.

The shared token is plain UDP authentication on a trusted LAN, **not encryption or HMAC**. A non-placeholder password/token is required, but firmware cannot prove WPA2 use or prevent a LAN sniffer learning a token. Use an isolated WPA2/WPA3 local network without Internet port forwarding. No unauthenticated web control endpoint is provided.
