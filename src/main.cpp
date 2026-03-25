#include <Arduino.h>

#include <easy_esc.h>

// ----------------------------------------------------------
// Super simple example:
// - 1 ESC motor object (Servo-style API)
// - 1 potentiometer controls throttle
// ----------------------------------------------------------

namespace
{
constexpr uint32_t kSerialBaud = 115200;

// Hardware pins (edit for your board):
constexpr gpio_num_t kMotor1Pin = GPIO_NUM_39; // ESC signal for M1
constexpr gpio_num_t kCurrentPin = GPIO_NUM_3; // ESC CRT/current pin (or GPIO_NUM_NC to disable)
constexpr uint8_t kPotPin = 4;                 // Analog potentiometer input pin

// ESC profile (for logs / documentation):
constexpr const char *kEscModelName = "Flycolor Raptor BLS-04 4-in-1";
constexpr const char *kEscFirmwareName = "X firmware (edit this to your exact firmware name)";

// ESC settings:
constexpr dshot_mode_t kDshotMode = DSHOT300; // Current project default
constexpr uint32_t kSignalTimeoutMs = 20000;
constexpr bool kHoldArmOnSignalTimeout = true; // Keep armed on timeout by forcing 0-throttle keepalive.
constexpr uint16_t kOutputRefreshMs = 10;
constexpr float kCurrentZeroOffsetMv = 0.0f;
constexpr float kCurrentMvPerAmp = 33.0f;
constexpr uint16_t kForceThrottleRaw = 0;       // 0 = use pot, otherwise force fixed raw throttle (48..2047).
constexpr uint16_t kBootSafePotThreshold = 120; // Pot must be near minimum before arming.
constexpr uint32_t kBootSafeHoldMs = 1000;      // How long pot must stay low.
constexpr uint32_t kBootSafePrintMs = 300;
constexpr uint32_t kArmSettleZeroMs = 1500;     // Send zero throttle for ESC sync after arm.

esc::EasyEscMotor MOTOR1(
    kMotor1Pin,
    kCurrentPin,
    kDshotMode,
    kSignalTimeoutMs,
    kOutputRefreshMs,
    kCurrentZeroOffsetMv,
    kCurrentMvPerAmp);

uint32_t gLastThrottleUpdateMs = 0;
uint32_t gLastPrintMs = 0;
uint16_t gLastThrottleRaw = 0;
uint16_t gStablePotCount = 0;
int gPrevPotRaw = -1;
uint32_t gLastTxErrorPrintMs = 0;
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    delay(300);

    Serial.println();
    Serial.println("[BOOT] EasyEscMotor potentiometer example");
    Serial.println("[BOOT] Remove propellers before testing.");
    Serial.printf("[BOOT] ESC model: %s\n", kEscModelName);
    Serial.printf("[BOOT] ESC firmware: %s\n", kEscFirmwareName);

    const bool motorPinOk = MOTOR1.isMotorPinSuitable();
    const bool currentPinOk = MOTOR1.isCurrentPinSuitable();
    Serial.printf("[BOOT] Pin check: motorPin=GPIO%d ok=%d, currentPin=GPIO%d ok=%d\n",
                  static_cast<int>(MOTOR1.pin()),
                  motorPinOk ? 1 : 0,
                  static_cast<int>(MOTOR1.currentPin()),
                  currentPinOk ? 1 : 0);
    if (!MOTOR1.arePinsSuitable())
    {
        Serial.println("[BOOT][WARN] Pin suitability check failed, but continuing.");
        Serial.println("[BOOT][WARN] begin() is the final authority (this avoids false negatives on some boards).");
    }

    analogReadResolution(12); // 0..4095
    pinMode(static_cast<int>(kPotPin), INPUT);
#if defined(ADC_11db)
    analogSetPinAttenuation(static_cast<uint8_t>(kPotPin), ADC_11db);
#endif

    if (!MOTOR1.begin())
    {
        Serial.printf("[BOOT] MOTOR1.begin() failed. status=%u\n", static_cast<unsigned>(MOTOR1.lastStatus()));
        while (true)
        {
            delay(1000);
        }
    }

    if (MOTOR1.hasCurrentMonitor())
    {
        // Optional: calibrate current-zero while motor is stopped.
        MOTOR1.calibrateCurrentZero(200, 2);
        Serial.printf("[BOOT] Current zero calibrated to %.2f mV\n", static_cast<double>(MOTOR1.currentZeroOffsetMv()));
    }

    MOTOR1.setHoldArmOnSignalTimeout(kHoldArmOnSignalTimeout);
    Serial.printf("[BOOT] holdArmOnSignalTimeout=%d timeoutMs=%lu refreshMs=%u\n",
                  MOTOR1.holdArmOnSignalTimeout() ? 1 : 0,
                  static_cast<unsigned long>(MOTOR1.timeoutMs()),
                  static_cast<unsigned>(MOTOR1.refreshMs()));

    Serial.printf("[BOOT] Safety gate: set pot near minimum (<= %u) for %lu ms before arm.\n",
                  static_cast<unsigned>(kBootSafePotThreshold),
                  static_cast<unsigned long>(kBootSafeHoldMs));
    uint32_t lowStartMs = 0;
    uint32_t lastBootPrintMs = 0;
    while (true)
    {
        MOTOR1.update();
        const uint32_t nowMs = millis();
        const int potRaw = analogRead(static_cast<int>(kPotPin));

        if (potRaw <= static_cast<int>(kBootSafePotThreshold))
        {
            if (lowStartMs == 0)
            {
                lowStartMs = nowMs;
            }
            if (nowMs - lowStartMs >= kBootSafeHoldMs)
            {
                break;
            }
        }
        else
        {
            lowStartMs = 0;
        }

        if (nowMs - lastBootPrintMs >= kBootSafePrintMs)
        {
            lastBootPrintMs = nowMs;
            const uint32_t stableMs = (lowStartMs == 0) ? 0 : (nowMs - lowStartMs);
            Serial.printf("[BOOT] waiting-safe-throttle pot=%d stable=%lu/%lu ms\n",
                          potRaw,
                          static_cast<unsigned long>(stableMs),
                          static_cast<unsigned long>(kBootSafeHoldMs));
        }

        delay(10);
    }

    MOTOR1.stop();

    if (!MOTOR1.arm())
    {
        Serial.printf("[BOOT] MOTOR1.arm() failed. status=%u\n", static_cast<unsigned>(MOTOR1.lastStatus()));
        while (true)
        {
            delay(1000);
        }
    }
    Serial.println("[BOOT] MOTOR1.arm() OK");

    // Some ESCs behave better if they receive a short zero-throttle stream right after arm.
    const uint32_t armStartMs = millis();
    while (millis() - armStartMs < kArmSettleZeroMs)
    {
        MOTOR1.update();
        MOTOR1.spinRaw(0);
        delay(10);
    }
    Serial.printf("[BOOT] Arm settle complete (%lu ms zero stream)\n", static_cast<unsigned long>(kArmSettleZeroMs));

    Serial.printf("[BOOT] Ready. DShot=%s motorPin=GPIO%d potPin=GPIO%d force_raw=%u\n",
                  MOTOR1.dshotModeName(),
                  static_cast<int>(MOTOR1.pin()),
                  static_cast<int>(kPotPin),
                  static_cast<unsigned>(kForceThrottleRaw));
}

void loop()
{
    MOTOR1.update();

    const uint32_t nowMs = millis();

    // Send throttle update at 100 Hz.
    if (nowMs - gLastThrottleUpdateMs >= 10)
    {
        gLastThrottleUpdateMs = nowMs;

        const int potRaw = analogRead(static_cast<int>(kPotPin)); // 0..4095
        long throttle = 0;
        if (kForceThrottleRaw > 0)
        {
            throttle = kForceThrottleRaw;
        }
        else
        {
            throttle = map(potRaw, 0, 4095, 0, 2047); // DShot raw range
        }

        if (throttle < 0)
        {
            throttle = 0;
        }
        if (throttle > 2047)
        {
            throttle = 2047;
        }

        uint16_t throttleRaw = static_cast<uint16_t>(throttle);
        if (throttleRaw > 0 && throttleRaw < esc::kDshotThrottleMinRaw)
        {
            throttleRaw = esc::kDshotThrottleMinRaw; // valid minimum running throttle
        }

        if (!MOTOR1.spinRaw(throttleRaw))
        {
            if (nowMs - gLastTxErrorPrintMs >= 300)
            {
                gLastTxErrorPrintMs = nowMs;
                Serial.printf("[ERR] spinRaw failed: status=%u rmt_code=%ld motor=%d pin=%d\n",
                              static_cast<unsigned>(MOTOR1.lastStatus()),
                              static_cast<long>(MOTOR1.lastRmtErrorCode()),
                              static_cast<int>(MOTOR1.lastRmtErrorMotor()),
                              static_cast<int>(MOTOR1.lastRmtErrorPin()));
            }
        }
        gLastThrottleRaw = throttleRaw;

        if (potRaw == gPrevPotRaw)
        {
            if (gStablePotCount < 1000)
            {
                ++gStablePotCount;
            }
        }
        else
        {
            gStablePotCount = 0;
            gPrevPotRaw = potRaw;
        }
    }

    // Print status at 4 Hz.
    if (nowMs - gLastPrintMs >= 250)
    {
        gLastPrintMs = nowMs;
        const int potRaw = analogRead(static_cast<int>(kPotPin));
        const esc::CurrentSample curr = MOTOR1.readCurrent();
        const float currentA = curr.amps;
        const float currentmA = curr.amps * 1000.0f;
        const uint16_t currentMv = curr.milliVolts;

        Serial.printf("pot=%d armed=%d status=%u thr_raw=%u curr_mv=%u curr_a=%.2f curr_ma=%.1f\n",
                      potRaw,
                      MOTOR1.isArmed() ? 1 : 0,
                      static_cast<unsigned>(MOTOR1.lastStatus()),
                      static_cast<unsigned>(gLastThrottleRaw),
                      static_cast<unsigned>(currentMv),
                      static_cast<double>(currentA),
                      static_cast<double>(currentmA));

        if (gStablePotCount > 40 && potRaw > 200 && potRaw < 3895)
        {
            Serial.println("[WARN] Pot ADC looks stuck. Check pot wiring and kPotPin.");
        }
    }
}
