# robo-hw v1

One UTF-8 JSON object per UDP datagram, port **4210**, at most **1024 bytes**. Send one request at a time and await its response. Do not retry a motion datagram with an old permit/sequence. An ambiguous timeout requires stopping and a fresh handshake. Keep the same UDP source IP and port for the session.

All requests have exactly these base fields:

```json
{"protocol":"robo-hw","version":1,"robot_id":"H1","token":"EXAMPLE_TOKEN_NOT_FOR_REAL_USE_123456","request_id":"req_1","type":"hello"}
```

`request_id` is 1–64 ASCII letters/digits/underscore/hyphen. Token is 24–64 characters in the same alphabet, matching ignored `include/secrets.h`. JSON bools and floats cannot stand in for integers. Duplicate root keys, unknown fields, embedded NULs, trailing JSON/garbage, invalid arrays and packets over the limit are rejected. An invalid packet from the established endpoint disarms the robot; incorrect tokens or another robot's ID do not acquire authority. `stop`/`estop` with the correct token and robot ID are honored immediately, even with an old optional sequence or malformed nonessential fields.

| Type | Additional fields | Effect |
|---|---|---|
| hello | none | Zero outputs, disarm, establish endpoint, new random permit |
| arm | boot_id, permit, seq | Consume permit younger than 250ms, arm with zero outputs; initial lease 250ms; return new permit |
| command | boot_id, permit, seq, ttl_ms, left_pwm, right_pwm, servo_us | Consume fresh permit, apply absolute output setpoints, return new permit |
| status | none | Read only; no permit renewal and no lease extension |
| stop | optional seq | Zero all outputs, disarm, invalidate permit |
| estop | optional seq | Zero all outputs, latch estop until reboot |

`boot_id` and `permit` are 32 lowercase hex characters generated on device. The boot ID remains unchanged until reboot; the permit changes after hello, arm, and each accepted command. Invalidated permits are returned as the empty string. `seq` is an integer in 1..2147483647, strictly increasing **across all sessions during one boot**; choose the next value from `response.seq + 1`. A successful hello does not reset sequence. Reboot is needed after sequence exhaustion.

`ttl_ms` is 1..250. The deadline is **permit issue time + ttl_ms**, using the device's wrap-safe monotonic clock. The previous output lease is checked before handling a new message. Once expired, a queued command cannot re-arm. A request whose permit has already aged beyond its requested TTL also stops the robot. Never calculate the deadline as receipt time + TTL.

`left_pwm/right_pwm` are integers in -96..96 by default. Positive means calibrated logical forward, negative reverse, zero coast. These are 8-bit PWM duty values, **not mm/s**. `servo_us` is exactly two integers; each is 0 (no pulse) or a configured safe absolute pulse width within 900..2100 microseconds. H1's second value must always be 0. Commands are idempotent setpoints: there is no blind "drop one more kit" operation.

Example command (replace boot/permit from the most recent response):

```json
{"protocol":"robo-hw","version":1,"robot_id":"H1","token":"EXAMPLE_TOKEN_NOT_FOR_REAL_USE_123456","request_id":"req_3","type":"command","boot_id":"00112233445566778899aabbccddeeff","permit":"ffeeddccbbaa99887766554433221100","seq":2,"ttl_ms":200,"left_pwm":40,"right_pwm":40,"servo_us":[0,0]}
```

Every reply to a valid authenticated request has the following keys. Invalid or unauthenticated datagrams can be silently dropped.

```json
{"protocol":"robo-hw","version":1,"type":"response","robot_id":"H1","request_id":"req_3","boot_id":"00112233445566778899aabbccddeeff","permit":"112233445566778899aabbccddeeff00","state":"armed","accepted":true,"reason":"ok","seq":2,"lease_remaining_ms":190,"left_pwm":40,"right_pwm":40,"servo_us":[0,0],"disc_present":false,"hardware_enabled":true,"uptime_ms":1000}
```

`state` is disarmed/armed/estop. The PWM/servo values report the applied logical output setpoints (before motor polarity inversion), not measured actuator position/speed. Outputs remain 0 with `hardware_enabled:false`. `disc_present` is a physical digital sensor reading for H1 and null for robots without a sensor; it is not proof that transport succeeded. `lease_remaining_ms` is the fresh permit lifetime after hello (250ms initially); while armed it is the smaller of permit lifetime and current output lease lifetime. Without a valid permit it is 0. It does not renew anything. `status` replies include the current permit without changing its issue time.

The shared token offers accidental/unauthorized-sender rejection on a trusted LAN, not cryptographic transport. The firmware requires a non-placeholder Wi-Fi password and token, but it cannot prove the access point uses WPA2 or prevent a LAN sniffer from learning a token. Use an isolated WPA2/WPA3 network. There is no unauthenticated web control endpoint.
