#include <assert.h>
#include <string.h>
#include "lease_state.h"
#include "json_envelope.h"

int main() {
  LeaseState s;
  assert(s.mode == LeaseState::Mode::Disarmed);
  assert(s.replyRemaining(100, 100, true) == 250); // hello reports fresh permit despite disarmed.
  assert(s.replyRemaining(100, 100, false) == 0);
  assert(s.replyRemaining(350, 100, true) == 0);
  assert(!s.command(1, 0, 250, 1));
  assert(s.arm(100, 1));
  assert(s.command(110, 100, 100, 2));
  assert(s.remaining(150) == 50); // Deadline is issue+TTL, not receive+TTL.
  assert(s.replyRemaining(150, 140, true) == 50); // Never overstate output lease.
  assert(!s.command(151, 150, 100, 2)); // Duplicate sequence rejected.
  assert(!s.command(200, 190, 100, 3)); // Expiry checked BEFORE renewal.
  assert(s.mode == LeaseState::Mode::Disarmed);
  assert(!s.command(201, 200, 100, 4)); // No automatic restart.
  assert(s.arm(202, 4));
  assert(!s.command(210, 200, 10, 5)); // Permit's deadline already elapsed.
  s.estop();
  s.disarm();
  assert(!s.arm(211, 6));
  assert(s.mode == LeaseState::Mode::Estop);
  LeaseState wrap;
  assert(wrap.arm(UINT32_MAX - 10, 1));
  assert(wrap.remaining(4) == 235); // millis() wrap-safe arithmetic.
  assert(wrap.tick(239));
  assert(wrap.remaining(239) == 0);
  LeaseState bounds;
  assert(bounds.arm(0, 1));
  assert(!bounds.command(1, 0, 0, 2));
  assert(!bounds.command(1, 0, 251, 2));
  assert(bounds.command(1, 1, 1, 2));
  assert(bounds.tick(2));
  const char* good = " {\"left_pwm\":1,\"servo_us\":[0,0]} \r\n";
  assert(envelopeShape(good, strlen(good), 2));
  const char* duplicate = "{\"seq\":1,\"seq\":2}";
  assert(!envelopeShape(duplicate, strlen(duplicate), 1));
  const char* trailing = "{\"seq\":1} {\"seq\":2}";
  assert(!envelopeShape(trailing, strlen(trailing), 1));
  const char* quoted = "{\"value\":\"}: [ : \\\"\"}";
  assert(envelopeShape(quoted, strlen(quoted), 1));
  assert(!envelopeShape("[]", 2, 0));
  assert(!envelopeShape("{}\0x", 4, 0));
  return 0;
}
