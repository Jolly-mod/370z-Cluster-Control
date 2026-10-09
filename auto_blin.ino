// 370Z gear display 1-6 (Uno + MCP2515, 16 MHz crystal, CS 10)
// Reads RPM + speed from the ECM (standard OBD-II requests),
// works out the gear from RPM / speed, and sends it to the cluster on 0x421.
//   Stopped            -> N
//   Moving, gear found -> 1..6 (M symbol = shift indicator, see CRUISE/PERF shift points)
//   Clutch in / coasting for 2 s -> N
// Serial Monitor at 115200 prints rpm, km/h, ratio and gear for tuning.

#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp(10);
struct can_frame tx421, req, rx;

// ---- settings ----
const float K = 28.97;              // rpm per km/h per gear ratio (stock 370Z tire size)
const float RATIO[6] = {3.794, 2.324, 1.624, 1.271, 1.000, 0.794};  // 6MT gear ratios
const float TOLERANCE = 0.10;       // ratio must be within 10% to count
const int   MIN_KMH_FOR_GEAR = 3;   // too coarse to judge below this
const unsigned long NO_MATCH_TO_N_MS = 2000;
const long  IDLE_LOW  = 700;        // idle RPM band, incl. the little flare when settling
const long  IDLE_HIGH = 1000;
const unsigned long IDLE_TO_N_MS = 2000;      // in idle band, throttle pressed: wait this long
const unsigned long IDLE_LIFT_TO_N_MS = 700;  // in idle band, foot off throttle: N this fast
const int   TPS_CLOSED_MARGIN = 3;  // % above the lowest throttle seen that still counts as "off"
const unsigned long GEAR_CONFIRM_MS = 400;    // a gear must match this long before it shows
const long  DROP_FROM_RPM = 1300;   // fast drop rule: RPM was at/above this...
const unsigned long DROP_WINDOW_MS = 1500;    // ...and fell into the idle band within this time = N now
// DEMO: test the shift indicator with the engine OFF (ignition ON). Fakes 3rd gear and
// sweeps a fake RPM 1000 -> 7500 -> 1000 every 33 s. SET BACK TO false BEFORE DRIVING.
const bool  DEMO_MODE = false;

// Shift indicator in the small spot next to the gear number (gear number always shows):
//   below shift point    -> M (or blank if SHOW_M_NORMALLY = false)
//   shift point and up   -> small "s" solid   (time to shift)
//   blink point and up   -> small "s" blinks  (right at redline)
// 0x421 byte 1: 00 = M, 01 = blank, 45 = small s
const bool  SHOW_M_NORMALLY = true;
// Two sets of shift points, picked by how hard you're pressing:
//   light throttle (under PERF_ON_PCT)  -> CRUISE points (early shifts)
//   hard throttle  (PERF_ON_PCT and up) -> PERF points (near redline, no nagging at 3k)
const long  CRUISE_SHIFT_RPM = 2000;
const long  CRUISE_BLINK_RPM = 2250;
const long  PERF_SHIFT_RPM   = 6500;
const long  PERF_BLINK_RPM   = 7000;
const int   PERF_ON_PCT  = 70;      // throttle % that switches to PERF
const int   PERF_OFF_PCT = 40;      // must drop below this...
const unsigned long PERF_HOLD_MS = 1500;   // ...for this long to go back to CRUISE
const unsigned long BLINK_MS = 150;     // on/off time when blinking
// Held too long: at/above OVERHOLD_RPM for OVERHOLD_MS -> gear number AND s both flash
const long  OVERHOLD_RPM = 6500;
const unsigned long OVERHOLD_MS = 5000;
const int   LAUNCH_END_KMH = 15;    // launch rule: show 1 when pulling away, until this speed...
const unsigned long LAUNCH_MAX_MS = 6000;     // ...or this long after leaving the stop
const int   REV_PIN = A0;
// Reverse source:
//   false = red TCM wire (pin 7), grounded in reverse, wired through a diode
//   true  = camera accessory wire, 12 V in reverse, wired through a 10k/3.3k divider
const bool  REV_ACTIVE_HIGH = false;
const unsigned long REV_DEBOUNCE_MS = 100;
// -------------------

long rpm = -1;
int kmh = -1;
int gearNum = 0;                    // 0 = N, 1..6 = gear
unsigned long last421 = 0, lastReq = 0, lastReply = 0, lastMatch = 0, lastPrint = 0;
float lastRatio = 0;
unsigned long idleSince = 0;        // 0 = not currently in idle band
int tps = -1;                       // throttle position, %
int tpsMin = 255;                   // lowest throttle seen = closed throttle
int tpsMax = 80;                    // highest seen (starts at a typical WOT reading)
bool perfMode = false;
unsigned long highSince = 0;        // when RPM went above OVERHOLD_RPM (0 = not above)
unsigned long perfLowSince = 0;
uint8_t pidStep = 0;
int candGear = 0;                   // gear the ratio currently points to
unsigned long candSince = 0;
unsigned long rpmHighAt = 0;        // last time RPM was at/above DROP_FROM_RPM
bool launchArmed = false;           // true after a stop, until the launch is over
unsigned long stoppedAt = 0;
bool blinkOn = true;
unsigned long lastBlink = 0;
bool inReverse = false;
bool revRaw = false;
unsigned long revChanged = 0;

uint8_t byte0For(int g) {
  if (g == 0) return 0x18;                 // N
  return 0x80 | ((g - 1) << 3);            // 1=80 2=88 3=90 4=98 5=A0 6=A8
}

// ---- sleep mode: stop transmitting when the car is off ----
const unsigned long SLEEP_AFTER_MS = 5000;   // ECM silent this long = car off
bool sleeping = false;

void setFilters(uint16_t mask, uint16_t id) {
  mcp.setFilterMask(MCP2515::MASK0, false, mask);
  mcp.setFilter(MCP2515::RXF0, false, id);
  mcp.setFilter(MCP2515::RXF1, false, id);
  mcp.setFilterMask(MCP2515::MASK1, false, mask);
  mcp.setFilter(MCP2515::RXF2, false, id);
  mcp.setFilter(MCP2515::RXF3, false, id);
  mcp.setFilter(MCP2515::RXF4, false, id);
  mcp.setFilter(MCP2515::RXF5, false, id);
}

void goAwake() {                       // normal: talk to the car, only hear ECM replies
  mcp.reset();
  mcp.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp.setConfigMode();
  setFilters(0x7FF, 0x7E8);
  mcp.setNormalMode();
  sleeping = false;
  Serial.println("awake");
}

void goSleep() {                       // silent: never transmit, hear everything
  mcp.reset();                         // also clears any frames stuck retrying
  mcp.setBitrate(CAN_500KBPS, MCP_16MHZ);
  mcp.setConfigMode();
  setFilters(0x000, 0x000);            // mask 0 = accept any ID
  mcp.setListenOnlyMode();
  sleeping = true;
  rpm = -1; kmh = -1; gearNum = 0;
  Serial.println("car off: sleeping (listen only)");
}

void setup() {
  Serial.begin(115200);
  pinMode(REV_PIN, REV_ACTIVE_HIGH ? INPUT : INPUT_PULLUP);
  goAwake();

  tx421.can_id = 0x421; tx421.can_dlc = 3;
  tx421.data[0] = 0x18; tx421.data[1] = 0x00; tx421.data[2] = 0x00;

  req.can_id = 0x7DF; req.can_dlc = 8;
  for (int i = 0; i < 8; i++) req.data[i] = 0x00;
  req.data[0] = 0x02; req.data[1] = 0x01;

  Serial.println("auto_gear_16 started");
}

void loop() {
  unsigned long now = millis();

  // Sleeping: send nothing, wake up as soon as any CAN traffic appears
  if (sleeping) {
    if (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
      goAwake();
      lastReply = millis();            // give the ECM a few seconds to answer
    }
    return;
  }
  if (now - lastReply > SLEEP_AFTER_MS) {
    goSleep();
    return;
  }

  // Ask for RPM, speed and throttle in turn
  if (now - lastReq >= 70) {
    lastReq = now;
    const uint8_t pids[4] = {0x0C, 0x0D, 0x0C, 0x11};   // RPM twice as often for the shift indicator
    req.data[2] = pids[pidStep];
    pidStep = (pidStep + 1) % 4;
    mcp.sendMessage(&req);
  }

  while (mcp.readMessage(&rx) == MCP2515::ERROR_OK) {
    if (rx.can_id == 0x7E8 && rx.data[1] == 0x41) {
      if (rx.data[2] == 0x0C) {
        uint16_t raw = ((uint16_t)rx.data[3] << 8) | rx.data[4];
        rpm = raw / 4; lastReply = now;
      } else if (rx.data[2] == 0x0D) {
        kmh = rx.data[3]; lastReply = now;
      } else if (rx.data[2] == 0x11) {
        tps = (rx.data[3] * 100) / 255; lastReply = now;
        if (tps < tpsMin) tpsMin = tps;      // learn what "foot off" reads on this car
        if (tps > tpsMax) tpsMax = tps;      // and what floored reads
      }
    }
  }

  // Track time spent at idle RPM
  bool inIdleBand = (rpm >= IDLE_LOW && rpm <= IDLE_HIGH);
  if (inIdleBand) { if (idleSince == 0) idleSince = now; }
  else idleSince = 0;

  bool throttleOff = (tps >= 0 && tps <= tpsMin + TPS_CLOSED_MARGIN);
  if (rpm >= DROP_FROM_RPM) rpmHighAt = now;
  bool fastDrop = inIdleBand && rpmHighAt != 0 && (now - rpmHighAt <= DROP_WINDOW_MS);

  // Work out the gear
  if (now - lastReply > 1000) {
    gearNum = 0;                                   // ECM silent: show N
  } else if (kmh >= 0 && kmh <= 1) {
    gearNum = 0;                                   // stopped
    stoppedAt = now; launchArmed = true;
  } else if (launchArmed && !throttleOff && tps >= 0 &&
             kmh < LAUNCH_END_KMH && now - stoppedAt <= LAUNCH_MAX_MS) {
    // Pulling away from a stop with throttle on: show 1 right away
    gearNum = 1; lastMatch = now; candGear = 0;
    if (inIdleBand) idleSince = now;               // don't let idle timers fire mid-launch
    rpmHighAt = 0;                                 // clutch biting isn't a "fast drop"
  } else if (inIdleBand) {
    // Idle-ish RPM while moving: don't guess a gear (stops 5/6 and low-gear flicker).
    // Foot off the throttle = almost certainly neutral/clutch in, so switch fast.
    // RPM fell fast from 1500+ straight to idle = clutch in / neutral: N right away.
    unsigned long need = throttleOff ? IDLE_LIFT_TO_N_MS : IDLE_TO_N_MS;
    if (fastDrop || now - idleSince >= need) gearNum = 0;
    candGear = 0;
  } else if (kmh >= MIN_KMH_FOR_GEAR && rpm > 500) {
    lastRatio = (float)rpm / kmh;
    int best = -1; float bestErr = 1e9;
    for (int g = 0; g < 6; g++) {
      float err = fabs(lastRatio / (K * RATIO[g]) - 1.0);
      if (err < bestErr) { bestErr = err; best = g; }
    }
    if (bestErr <= TOLERANCE) {
      // Only show a gear once the ratio has pointed at it steadily
      if (best + 1 != candGear) { candGear = best + 1; candSince = now; }
      if (now - candSince >= GEAR_CONFIRM_MS) gearNum = candGear;
      lastMatch = now;
    } else {
      candGear = 0;
    }
  }
  if (launchArmed && kmh > 1 &&
      (throttleOff || kmh >= LAUNCH_END_KMH || now - stoppedAt > LAUNCH_MAX_MS))
    launchArmed = false;                           // launch over: normal logic from here
  if (gearNum > 0 && now - lastMatch > NO_MATCH_TO_N_MS) gearNum = 0;  // clutch in / coasting

  // Demo override: fake RPM sweep in 3rd gear, keep awake even if the ECM is quiet
  if (DEMO_MODE) {
    // 33 s cycle: climb 1000->7500 over 12 s, hold 7500 for 7 s (long enough for the
    // 5 s "held too long" flash), drop back over 12 s, sit at 1000 for 2 s
    unsigned long t = now % 33000;
    if (t < 12000)      rpm = 1000 + (long)(t * 6500UL / 12000UL);
    else if (t < 19000) rpm = 7500;
    else if (t < 31000) rpm = 7500 - (long)((t - 19000) * 6500UL / 12000UL);
    else                rpm = 1000;
    // alternate cycles: light throttle (cruise points), then floored (perf points)
    bool hard = ((now / 33000) % 2) == 1;
    tpsMin = 10; tpsMax = 80;
    tps = hard ? 78 : 25;
    gearNum = 3;
    lastReply = now;
  }

  // Reverse switch (debounced). Reverse always wins over the speed-based gear.
  bool r = (digitalRead(REV_PIN) == (REV_ACTIVE_HIGH ? HIGH : LOW));
  if (r != revRaw) { revRaw = r; revChanged = now; }
  if (now - revChanged >= REV_DEBOUNCE_MS) inReverse = revRaw;

  // Shift indicator: decide whether the M symbol is lit
  if (now - lastBlink >= BLINK_MS) { lastBlink = now; blinkOn = !blinkOn; }
  // Throttle as 0-100% of this car's real range (closed..floored)
  int thrPct = 0;
  if (tps >= 0 && tpsMax > tpsMin) thrPct = constrain((tps - tpsMin) * 100 / (tpsMax - tpsMin), 0, 100);
  if (thrPct >= PERF_ON_PCT) { perfMode = true; perfLowSince = 0; }
  else if (perfMode && thrPct < PERF_OFF_PCT) {
    if (perfLowSince == 0) perfLowSince = now;
    if (now - perfLowSince >= PERF_HOLD_MS) perfMode = false;
  } else perfLowSince = 0;
  long shiftAt = perfMode ? PERF_SHIFT_RPM : CRUISE_SHIFT_RPM;
  long blinkAt = perfMode ? PERF_BLINK_RPM : CRUISE_BLINK_RPM;

  // Track how long we've been held at high RPM in gear
  if (gearNum > 0 && !inReverse && rpm >= OVERHOLD_RPM) { if (highSince == 0) highSince = now; }
  else highSince = 0;
  bool flashAll = (highSince != 0 && now - highSince >= OVERHOLD_MS);

  uint8_t smallSpot = SHOW_M_NORMALLY ? 0x00 : 0x01;     // M or blank
  bool sLit = false;
  if (gearNum > 0 && !inReverse) {
    if (rpm >= blinkAt) { sLit = blinkOn; smallSpot = blinkOn ? 0x45 : 0x01; }
    else if (rpm >= shiftAt) { sLit = true; smallSpot = 0x45; }
  }

  // Send 0x421 every 50 ms
  if (now - last421 >= 50) {
    last421 = now;
    if (inReverse) {
      tx421.data[0] = 0x10;                       // R
      tx421.data[1] = 0x00;
    } else {
      if (flashAll) {
        // whole display flashes: gear + s, then blank
        tx421.data[0] = blinkOn ? byte0For(gearNum) : 0x00;
        tx421.data[1] = blinkOn ? 0x45 : 0x01;
      } else {
        tx421.data[0] = byte0For(gearNum);
        tx421.data[1] = (gearNum > 0) ? smallSpot : 0x00;
      }
    }
    mcp.sendMessage(&tx421);
  }

  if (now - lastPrint >= 500) {
    lastPrint = now;
    if (DEMO_MODE) Serial.print("[DEMO] ");
    Serial.print("rpm ");   Serial.print(rpm);
    Serial.print("  kmh "); Serial.print(kmh);
    Serial.print("  ratio "); Serial.print(lastRatio, 1);
    Serial.print("  tps "); Serial.print(tps);
    Serial.print(throttleOff ? "(off)" : "");
    Serial.print(perfMode ? "  PERF" : "  CRUISE");
    Serial.print(flashAll ? "  [FLASH ALL]" : (sLit ? "  [s]" : ""));
    Serial.print("  gear ");
    if (inReverse) Serial.println("R");
    else if (gearNum == 0) Serial.println("N");
    else Serial.println(gearNum);
  }
}
