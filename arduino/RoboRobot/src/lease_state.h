#pragma once
#include <stdint.h>

// No Arduino dependency: tests/native/lease_state_test.cpp exercises this core.
struct LeaseState {
  enum class Mode { Disarmed, Armed, Estop };
  Mode mode = Mode::Disarmed;
  uint32_t start = 0;
  uint32_t ttl = 0;
  int32_t seq = 0;
  bool expired(uint32_t now) const {
    return mode == Mode::Armed && uint32_t(now - start) >= ttl;
  }
  bool tick(uint32_t now) {
    if (!expired(now)) return false;
    disarm();
    return true;
  }
  void disarm() { if (mode != Mode::Estop) mode = Mode::Disarmed; ttl = 0; }
  void estop() { mode = Mode::Estop; ttl = 0; }
  bool arm(uint32_t now, int32_t nextSeq) {
    if (mode != Mode::Disarmed || nextSeq <= seq) return false;
    mode = Mode::Armed; start = now; ttl = 250; seq = nextSeq; return true;
  }
  bool command(uint32_t now, uint32_t issued, uint32_t nextTtl, int32_t nextSeq) {
    tick(now); // Expiry must be handled before any possible renewal.
    if (mode != Mode::Armed || nextSeq <= seq || nextTtl < 1 || nextTtl > 250 || uint32_t(now - issued) >= nextTtl) return false;
    start = issued; ttl = nextTtl; seq = nextSeq; return true;
  }
  uint32_t remaining(uint32_t now) const {
    return mode == Mode::Armed && !expired(now) ? ttl - uint32_t(now - start) : 0;
  }
  uint32_t replyRemaining(uint32_t now, uint32_t issued, bool hasPermit) const {
    const uint32_t age = uint32_t(now - issued);
    const uint32_t permitRemaining = hasPermit && age < 250 ? 250 - age : 0;
    if (mode != Mode::Armed) return permitRemaining;
    const uint32_t outputRemaining = remaining(now);
    return outputRemaining < permitRemaining ? outputRemaining : permitRemaining;
  }
  const char* name() const {
    return mode == Mode::Armed ? "armed" : (mode == Mode::Estop ? "estop" : "disarmed");
  }
};
