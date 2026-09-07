# 통신 규격 v1 · 4대 가짜 수신기 시험

2026-09-07. 하드웨어가 미정인 상태에서 사용할 **소프트웨어 참조 규격과 시험 구현**이다.
보드/배선/통신 매체를 확정하거나 실제 로봇 운용을 승인한 문서가 아니다.
대상은 H1/H2/B1/B2의 2륜 차동 차체 명령이며 메카넘 옆 이동, PWM, 엔코더 측정은 없다.

## 구현 범위와 실행

- `wire_codec.py`: 제한된 JSON 바이트 프레임과 분할/합쳐진 수신 처리.
- `wire_sender.py`: 기존 차동 제어 패킷 전체 검증 → 로봇별 명령/응답 연결.
- `fake_receiver.py`: 장치 시계, 부팅/연결 식별, 한 번 쓰는 허가, 독립 만료를 모델링한 수신기.
- `wire_lab.py`: 서로 다른 시계 원점을 가진 4대의 양방향 바이트 루프와 장애 주입 시험.

```powershell
.\.venv\Scripts\python.exe -m robo_control.wire_lab --compact
.\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_wire*.py' -v
.\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_fake_receiver.py' -v
```

실행 결과의 `passed=true`는 소프트웨어 시험 통과다. `transport=in_memory_framed_bytes`,
`synthetic=true`, `device_io=false`, `hardware_ready=false`, `motion_permitted=false`를 유지한다.
가상 시각을 진행시키는 시험이며 실시간 OS 스케줄링·실제 무선 통신 시험이 아니다.
기존 차동 제어기에 고정된 합성 관측을 넣지만, 명령을 적분해 로봇을 이동시키거나 득점하지 않는다.

기존 `runtime`의 기본 출력은 계속 `MockActuatorBank`다. 새 송신기는 그 출력 패킷을
받는 별도 API이며 `runtime --mission` 출력과의 통합 시험을 추가했다. 실제 송수신을
자동으로 켜는 CLI 옵션은 추가하지 않았다. 기존 `adapters.UdpRobotTransport`는 다른
고수준 스키마의 단방향 전송기이며, 이 규격의 ACK/만료 검증기로 간주하거나 바로 연결하지 않는다.

## 바이트 형식

- UTF-8 JSON 객체 한 개 + LF. CRLF도 허용하며 한 프레임은 종결자를 포함해 최대 4096바이트.
- 중첩 최대 16단계, 정수 signed64 범위. 명령 `seq`는 상호운용을 위해 1..2^53-1로 더 제한한다.
- 중복 JSON 키, NaN/Infinity, 잘못된 UTF-8, JSON 객체 외 최상위 값, 알 수 없는 명령 필드는 거부한다.
- 버전/순서/TTL에 `true`를 숫자 1처럼 사용하는 것도 거부한다.
- 바이트가 나뉘어 도착해도 LF까지 명령을 실행하지 않는다. 오류 프레임이 있는 `feed()` 호출은
  그 호출의 결과 전체를 반환하지 않고 잠긴다. 이전 호출에서 처리한 명령을 취소한 것은 아니다.
- 손상/초과/불완전 EOF이면 호출자가 해당 연결을 정지 처리한다. decoder `reset()`은 버퍼만
  초기화하며 로봇을 재가동하지 않는다. 별도의 새 hello/arm이 필요하다.

## 요청 메시지

모든 요청의 공통 필드: `protocol="robo-wire"`, `version=1`, `type`, `robot_id`.
`host_session_id`, `hello_id`, `boot_id`, `link_id`, `challenge_id`, `permit_id`는
영문/숫자/밑줄/하이픈으로 된 1..128자 토큰이다.

| type | 공통 필드 외 필수 필드 | 의미 |
|---|---|---|
| `hello` | `host_session_id`, `hello_id` | 기존 출력을 0으로 만들고 새 연결 challenge 요청 |
| `arm` | 연결 문맥, `challenge_id` | 유효 challenge 확인 후 **0 출력으로만** 준비 |
| `drive` | 연결 문맥, `permit_id`, `ttl_ms`, `v_mm_s`, `omega_rad_s` | 신선한 한 번의 차체 목표 속도 |
| `stop` | 연결 문맥, `reason` | 출력 0, 허가 폐기, 재시작에 새 handshake 필요 |
| `estop` | 연결 문맥, `reason` | 출력 0 및 비상정지 잠금 |
| `status` | 연결 문맥 | 상태 조회만 수행; 수명/허가를 갱신하지 않음 |

연결 문맥은 `host_session_id`, `boot_id`, `link_id`, `seq` 네 필드다.
`reason`은 1..128자 문자열이다. 알려진 스키마에 없는 필드는 거부한다.
`drive` 예시(토큰은 실제 해당 handshake에서 받은 것으로 대체):

```json
{"protocol":"robo-wire","version":1,"type":"drive","robot_id":"H1","host_session_id":"host-run-1","boot_id":"receiver-boot","link_id":"receiver-link","seq":2,"permit_id":"one-use-permit","ttl_ms":300,"v_mm_s":50.0,"omega_rad_s":0.25}
```

`v_mm_s` 양수는 차체 전진, 음수는 후진이다. `omega_rad_s` 양수는 반시계 회전이다.
옆 속도나 경기장 목표 좌표, PC 절대/단조 시각은 wire 명령에 보내지 않는다.
v1 소프트웨어 시험 상한은 |v|≤180mm/s, |ω|≤1.5rad/s, TTL 1..300ms다.
수신기 설정은 이 한도를 낮출 수만 있다. 실물에 적합한 속도라는 인증은 아니다.

## 서로 다른 시계에서 오래된 명령을 막는 방법

PC `issued_at_s`를 보드의 단조 시각과 직접 빼지 않는다. 수신 시점부터 무조건 300ms를
주는 것도 금지한다. 그러면 오래 지연된 명령이 도착 후 새 수명을 얻을 수 있기 때문이다.

1. `hello` 응답은 수신기가 새 `boot_id/link_id/challenge_id`를 제시한다. challenge는
   수신기 시계로 최대 300ms만 유효하다. `hello` 재전송도 출력은 0이며 새 연결을 만든다.
2. `arm`은 유효 challenge를 소비하고 0 출력 상태에서 `permit_id`를 발급한다.
3. 송신기는 이 허가 응답을 받은 **PC 시각 이후에 생성된 새 제어 패킷**만 보낸다.
   기존 패킷을 새 허가로 다시 포장하지 않는다. 원본 session/sequence/TTL도 PC에서 검사한다.
4. 수신기는 `drive`의 만료를 **허가 발급 시각 + ttl_ms**로 고정한다. 수신/ACK/재전송 시각으로
   밀지 않는다. 경계 시각에 도달하면 이미 만료다. 원본 TTL은 ms로 내림 변환한다.
5. 정상 수신은 다음 명령용 새 허가를 발급하지만 현재 출력의 만료 시각을 연장하지 않는다.
   새 명령을 처리하기 전에도 이전 출력의 watchdog을 먼저 검사한다. 이미 만료됐으면
   지연 명령으로 복구할 수 없고 명시적인 새 hello/arm이 필요하다.

3번의 선후관계 때문에 허가는 원본 제어 명령보다 먼저 존재한다. 따라서 허가에 매인
수신기 만료가 원본 명령의 TTL보다 뒤로 늘어나지 않는다. PC와 보드의 시계 원점은
달라도 되지만 각 시계는 단조이고 실제 시간 진행을 적절히 측정해야 한다. 하드웨어
시계 오차·감시 주기·스케줄링 지연 검증은 별도다. RTT가 TTL에 가까우면 정지/재연결이
빈번해질 수 있으며, 이를 피하려고 TTL을 임의로 늘리지 않는다.

## 응답, 중복, 정지

응답의 고정 필드:

```text
protocol, version, type="response", robot_id,
host_session_id, boot_id, link_id, hello_id,
request_type, seq, result, reason, state,
challenge_id, permit_id, lease_remaining_ms, v_mm_s, omega_rad_s,
synthetic=true, execution="unknown",
device_io=false, hardware_ready=false, motion_permitted=false
```

`result`는 `challenge/accepted/rejected/status`, `state`는 `disarmed/armed/estop`다.
hello의 `seq`와 아직 없는 토큰은 null이다. 모든 응답은 현재 handshake의 `hello_id`를
유지한다. `lease_remaining_ms`는 허가/challenge의 진단값이며 수신 명령의 새 TTL이 아니다.
응답 속도는 가짜 수신기에 저장한 **목표값**이지 측정 속도가 아니다.

- `accepted`는 파싱·식별·검증을 거쳐 가짜 수신기에 반영됐다는 뜻이다.
  실제 구동 완료/집기 성공/센서 검증은 전부 unknown이다. 응답에 센서 `signals`를 만들지 않는다.
- 로봇별 한 요청만 응답 대기한다. 잘못된 로봇/세션/부팅/연결/명령/hello ID의 ACK와
  중복/늦은 ACK는 새 허가를 주지 않는다. 기본 ACK 제한은 PC 시계 300ms 이내다.
- ACK 유실은 결과 불명이다. 자동 재전송·자동 재연결·자동 비상정지 해제를 하지 않는다.
  같은 호스트 세션의 sender를 재사용해 원본 명령 순서 기록을 유지한다.
- 중복/역순/소비한 permit/잘못된 현 연결 메시지는 거부하고 해당 수신기를 0·비준비로 만든다.
  유효한 다른 로봇 ID의 패킷은 그 로봇의 순서/출력을 바꾸지 않는다.
- 현 연결의 stop/estop은 오래된 seq라도 정지를 우선한다. 이전 연결의 명령으로는
  새 연결을 가동할 수 없다. 오래된 메시지가 정지를 유발하는 보수적 동작은 허용한다.
- 비상정지는 hello/일반 stop으로 풀리지 않는다. 가짜 수신기의 로컬
  `reset_emergency_stop()` 후에도 새 handshake와 새 명령이 필요하다. wire reset은 없다.

## 호출자 책임과 남은 작업

```python
sender = WireCommandSender("H1", session_id=controller.session_id)
receiver = FakeRobotReceiver("H1")  # 시험용, 실제 장치가 아님
# hello -> challenge -> arm -> ACK를 encode_frame/decoder로 왕복한다.
# ACK마다 sender.accept_response(response, host_now)를 호출한다.
# 그 이후 생성한 controller_packet으로 sender.drive_from_packet(packet, host_now).
# 새 입력이 없어도 sender.poll(host_now), receiver.tick(receiver_now)를 각각 계속 호출한다.
```

- 가짜 수신기는 실행자가 `tick()`을 계속 호출할 때만 독립 만료를 모델링한다.
  같은 Python 프로세스가 멈췄는데 실물 모터도 정지한다고 보장하지 않는다.
- 송신 API 검증 실패/ACK 거부/시간 초과는 운용 감시자가 정지·연결 폐기로 처리해야 한다.
  한 로봇의 응답 실패를 전체 임무에 어떻게 전파할지, 연결된 로봇들의 최선의 정지 송신,
  명시적 재무장 절차는 실제 runtime 전송 계층 연결 시 구현할 항목이다.
- 4대 패킷 전체를 검증한 뒤 로봇별 메시지를 만들 수 있지만 네 장치가 동시에 수신/동작하는
  분산 원자성을 제공하지 않는다. 다른 로봇의 ACK를 대신 사용하지 않는다.
- 이 토큰은 신선도·중복 방지 계약이지 인증 암호가 아니다. 실제 TCP/UDP/직렬/BLE 등을
  선택한 뒤 전송 프레이밍 대응, 인증/무결성·접근 제어·주소 매핑을 결정해야 한다.
- 집게/게이트/키트 배출과 센서 telemetry는 v1에서 지원하지 않고 모르는 명령으로 거부한다.
  부품 결정 뒤 별도 명령 ID·중복 조작 방지·센서 근거/호스트 시각 변환 규격을 확장한다.
  현재 응답을 `runtime`의 실제 조작 센서 피드백으로 변환하면 안 된다.
- 펌웨어의 독립 타이머·부팅 시 정지·모터 enable·전원/물리 비상정지와 제한된 벤치 시험이 필요하다.

## 검증 기록

전용 시험은 프레이밍, 송신기, 수신기, 실제 기존 제어기/임무 runtime 출력 연결을 포함한다.
네 수신기의 서로 다른 시계 원점, 지연/누락/중복/역순/손상/분할 메시지, ACK 유실·늦은 ACK,
재접속/재부팅/비상정지와 관측 유실 시 stop 전달을 검사한다.

2026-09-07 로컬 검증 결과:

| 검증 | 결과 |
|---|---|
| 전체 Python 회귀시험, `.venv`/OpenCV 5 | **488개 통과** |
| 이번 추가 시험 | **90개 통과** — codec 30, sender 28, receiver 22, integration 10 |
| OpenCV 없는 기본 Python의 추가 시험 | 90개 통과, 건너뜀 없음 |
| `wire_lab --compact` | 7개 장애/송수신 검사 통과, 최종 4대 0 |
| 신규 Python 파일 Ruff / `git diff --check` | 통과 |

CI에 이 데모와 패키지 설치 후 실행 검사를 추가했다. 위 수치는 로컬 검증 결과이며,
push 후 원격 CI 결과는 별도로 확인해야 한다. 실제 네트워크/직렬/카메라/장치 동작은 수행하지 않았다.
