#include "esc_dshot_output.h"

namespace esc
{
bool EscDshotOutput::isConfigValid(const DshotOutputConfig &config)
{
    if (config.motorCount == 0 || config.motorCount > kMaxMotors)
    {
        return false;
    }
    if (config.throttleMinRaw < kDshotThrottleMinRaw || config.throttleMaxRaw > kDshotThrottleMaxRaw)
    {
        return false;
    }
    if (config.throttleMinRaw > config.throttleMaxRaw)
    {
        return false;
    }

    for (uint8_t i = 0; i < config.motorCount; ++i)
    {
        if (config.motorPins[i] == GPIO_NUM_NC)
        {
            return false;
        }
    }
    return true;
}

uint16_t EscDshotOutput::clampRawThrottle(uint16_t valueRaw, const DshotOutputConfig &config)
{
    if (valueRaw == 0)
    {
        return 0;
    }
    if (valueRaw < config.throttleMinRaw)
    {
        return config.throttleMinRaw;
    }
    if (valueRaw > config.throttleMaxRaw)
    {
        return config.throttleMaxRaw;
    }
    return valueRaw;
}

Status EscDshotOutput::begin(const DshotOutputConfig &config)
{
    if (!isConfigValid(config))
    {
        return Status::InvalidArg;
    }

    _config = config;
    _initialized = false;
    _armed = false;
    _passthroughActive = false;
    _lastUpdateMs = millis();
    _lastRefreshMs = _lastUpdateMs;
    _lastDriverErrorCode = 0;
    _lastDriverErrorMotor = -1;
    _lastDriverErrorPin = GPIO_NUM_NC;

    for (uint8_t i = 0; i < kMaxMotors; ++i)
    {
        _lastThrottleRaw[i] = 0;
        _driverSuspended[i] = false;
        clearDriver(i);
    }

    for (uint8_t i = 0; i < _config.motorCount; ++i)
    {
        const Status status = createDriver(i);
        if (status != Status::Ok)
        {
            return status;
        }
    }

    _initialized = true;
    return stopAllMotors();
}

bool EscDshotOutput::isInitialized() const
{
    return _initialized;
}

Status EscDshotOutput::arm()
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }
    if (_passthroughActive)
    {
        return Status::PassthroughActive;
    }
    _armed = true;
    _lastUpdateMs = millis();
    return Status::Ok;
}

Status EscDshotOutput::disarm()
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }
    _armed = false;
    return stopAllMotors();
}

bool EscDshotOutput::isArmed() const
{
    return _armed;
}

Status EscDshotOutput::setMotorThrottleRaw(uint8_t motor, uint16_t valueRaw)
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }
    if (motor >= _config.motorCount)
    {
        return Status::InvalidArg;
    }
    if (_passthroughActive)
    {
        return Status::PassthroughActive;
    }
    if (!_armed)
    {
        return Status::NotArmed;
    }

    const uint16_t clamped = clampRawThrottle(valueRaw, _config);
    const Status sendStatus = sendMotorRaw(motor, clamped);
    if (sendStatus != Status::Ok)
    {
        return sendStatus;
    }

    _lastThrottleRaw[motor] = clamped;
    _lastUpdateMs = millis();
    _lastRefreshMs = _lastUpdateMs;
    return Status::Ok;
}

Status EscDshotOutput::setAllMotorsThrottleRaw(uint16_t valueRaw)
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }
    if (_passthroughActive)
    {
        return Status::PassthroughActive;
    }
    if (!_armed)
    {
        return Status::NotArmed;
    }

    const uint16_t clamped = clampRawThrottle(valueRaw, _config);
    for (uint8_t i = 0; i < _config.motorCount; ++i)
    {
        if (_driverSuspended[i])
        {
            continue;
        }

        const Status sendStatus = sendMotorRaw(i, clamped);
        if (sendStatus != Status::Ok)
        {
            return sendStatus;
        }
        _lastThrottleRaw[i] = clamped;
    }

    _lastUpdateMs = millis();
    _lastRefreshMs = _lastUpdateMs;
    return Status::Ok;
}

Status EscDshotOutput::stopAllMotors()
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }

    for (uint8_t i = 0; i < _config.motorCount; ++i)
    {
        _lastThrottleRaw[i] = 0;
        if (_driverSuspended[i])
        {
            continue;
        }

        const Status sendStatus = sendMotorRaw(i, 0);
        if (sendStatus != Status::Ok)
        {
            return sendStatus;
        }
    }

    _lastUpdateMs = millis();
    _lastRefreshMs = _lastUpdateMs;
    return Status::Ok;
}

void EscDshotOutput::service()
{
    if (!_initialized || !_armed || _passthroughActive)
    {
        return;
    }
    const uint32_t nowMs = millis();
    if (_config.signalTimeoutMs > 0 && (nowMs - _lastUpdateMs) > _config.signalTimeoutMs)
    {
        if (_config.holdArmOnSignalTimeout)
        {
            // Keep ESC link alive at zero throttle without requiring re-arm.
            stopAllMotors();
            return;
        }
        _armed = false;
        stopAllMotors();
        return;
    }

    if (_config.outputRefreshMs > 0 && (nowMs - _lastRefreshMs) >= _config.outputRefreshMs)
    {
        sendAllStored();
        _lastRefreshMs = nowMs;
    }
}

Status EscDshotOutput::setPassthroughActive(bool active)
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }

    if (active)
    {
        _armed = false;
        const Status stopStatus = stopAllMotors();
        if (stopStatus != Status::Ok)
        {
            return stopStatus;
        }
    }

    _passthroughActive = active;
    _lastUpdateMs = millis();
    _lastRefreshMs = _lastUpdateMs;
    return Status::Ok;
}

bool EscDshotOutput::isPassthroughActive() const
{
    return _passthroughActive;
}

uint8_t EscDshotOutput::motorCount() const
{
    return _config.motorCount;
}

gpio_num_t EscDshotOutput::motorPin(uint8_t motor) const
{
    if (motor >= _config.motorCount)
    {
        return GPIO_NUM_NC;
    }
    return _config.motorPins[motor];
}

dshot_mode_t EscDshotOutput::dshotMode() const
{
    return _config.dshotMode;
}

void EscDshotOutput::setSignalTimeoutMs(uint32_t timeoutMs)
{
    _config.signalTimeoutMs = timeoutMs;
}

uint32_t EscDshotOutput::signalTimeoutMs() const
{
    return _config.signalTimeoutMs;
}

void EscDshotOutput::setHoldArmOnSignalTimeout(bool enabled)
{
    _config.holdArmOnSignalTimeout = enabled;
}

bool EscDshotOutput::holdArmOnSignalTimeout() const
{
    return _config.holdArmOnSignalTimeout;
}

void EscDshotOutput::setOutputRefreshMs(uint16_t refreshMs)
{
    _config.outputRefreshMs = refreshMs;
}

uint16_t EscDshotOutput::outputRefreshMs() const
{
    return _config.outputRefreshMs;
}

int32_t EscDshotOutput::lastDriverErrorCode() const
{
    return _lastDriverErrorCode;
}

int8_t EscDshotOutput::lastDriverErrorMotor() const
{
    return _lastDriverErrorMotor;
}

gpio_num_t EscDshotOutput::lastDriverErrorPin() const
{
    return _lastDriverErrorPin;
}

Status EscDshotOutput::suspendMotorDriverForPassthrough(uint8_t motor)
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }
    if (motor >= _config.motorCount)
    {
        return Status::InvalidArg;
    }
    if (_driverSuspended[motor])
    {
        return Status::Ok;
    }

    const Status sendStatus = sendMotorRaw(motor, 0);
    if (sendStatus != Status::Ok)
    {
        return sendStatus;
    }

    clearDriver(motor);
    _driverSuspended[motor] = true;
    _lastThrottleRaw[motor] = 0;
    return Status::Ok;
}

Status EscDshotOutput::resumeMotorDriverFromPassthrough(uint8_t motor)
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }
    if (motor >= _config.motorCount)
    {
        return Status::InvalidArg;
    }
    if (!_driverSuspended[motor])
    {
        return Status::Ok;
    }

    const Status createStatus = createDriver(motor);
    if (createStatus != Status::Ok)
    {
        return createStatus;
    }

    _driverSuspended[motor] = false;
    _lastThrottleRaw[motor] = 0;
    return sendMotorRaw(motor, 0);
}

Status EscDshotOutput::createDriver(uint8_t motor)
{
    if (motor >= _config.motorCount)
    {
        return Status::InvalidArg;
    }

    const gpio_num_t pin = _config.motorPins[motor];
    Serial.printf("[RMT] attach motor=%u gpio=%d mode=%u\n",
                  static_cast<unsigned>(motor),
                  static_cast<int>(pin),
                  static_cast<unsigned>(_config.dshotMode));

    _drivers[motor] = std::make_unique<DShotRMT>(pin, _config.dshotMode);
    if (!_drivers[motor])
    {
        _lastDriverErrorCode = -1;
        _lastDriverErrorMotor = static_cast<int8_t>(motor);
        _lastDriverErrorPin = pin;
        Serial.printf("[RMT] attach FAILED motor=%u gpio=%d code=%d (alloc)\n",
                      static_cast<unsigned>(motor),
                      static_cast<int>(pin),
                      static_cast<int>(_lastDriverErrorCode));
        return Status::DriverError;
    }

    const dshot_result_t initResult = _drivers[motor]->begin();
    if (!initResult.success)
    {
        _lastDriverErrorCode = static_cast<int32_t>(initResult.result_code);
        _lastDriverErrorMotor = static_cast<int8_t>(motor);
        _lastDriverErrorPin = pin;
        Serial.printf("[RMT] attach FAILED motor=%u gpio=%d code=%d\n",
                      static_cast<unsigned>(motor),
                      static_cast<int>(pin),
                      static_cast<int>(_lastDriverErrorCode));
        clearDriver(motor);
        return Status::DriverError;
    }

    const bool reverseDirection = _config.motorDirectionReversed[motor];
    const dshot_result_t directionResult = _drivers[motor]->setMotorSpinDirection(reverseDirection);
    if (!directionResult.success)
    {
        _lastDriverErrorCode = static_cast<int32_t>(directionResult.result_code);
        _lastDriverErrorMotor = static_cast<int8_t>(motor);
        _lastDriverErrorPin = pin;
        Serial.printf("[RMT] direction FAILED motor=%u gpio=%d code=%d reversed=%u\n",
                      static_cast<unsigned>(motor),
                      static_cast<int>(pin),
                      static_cast<int>(_lastDriverErrorCode),
                      static_cast<unsigned>(reverseDirection));
        clearDriver(motor);
        return Status::DriverError;
    }

    Serial.printf("[RMT] attach OK motor=%u gpio=%d\n",
                  static_cast<unsigned>(motor),
                  static_cast<int>(pin));
    return Status::Ok;
}

void EscDshotOutput::clearDriver(uint8_t motor)
{
    if (motor < _drivers.size())
    {
        _drivers[motor].reset();
    }
}

Status EscDshotOutput::sendMotorRaw(uint8_t motor, uint16_t valueRaw)
{
    if (motor >= _config.motorCount)
    {
        return Status::InvalidArg;
    }
    if (!_drivers[motor])
    {
        return Status::DriverError;
    }

    const dshot_result_t txResult = _drivers[motor]->sendThrottle(valueRaw);
    if (!txResult.success)
    {
        _lastDriverErrorCode = static_cast<int32_t>(txResult.result_code);
        _lastDriverErrorMotor = static_cast<int8_t>(motor);
        _lastDriverErrorPin = _config.motorPins[motor];
        Serial.printf("[RMT] tx FAILED motor=%u gpio=%d code=%d throttle=%u\n",
                      static_cast<unsigned>(motor),
                      static_cast<int>(_lastDriverErrorPin),
                      static_cast<int>(_lastDriverErrorCode),
                      static_cast<unsigned>(valueRaw));
        return Status::DriverError;
    }

    return Status::Ok;
}

Status EscDshotOutput::sendAllStored()
{
    if (!_initialized)
    {
        return Status::NotInitialized;
    }

    for (uint8_t i = 0; i < _config.motorCount; ++i)
    {
        if (_driverSuspended[i])
        {
            continue;
        }
        const Status s = sendMotorRaw(i, _lastThrottleRaw[i]);
        if (s != Status::Ok)
        {
            return s;
        }
    }
    return Status::Ok;
}
} // namespace esc
