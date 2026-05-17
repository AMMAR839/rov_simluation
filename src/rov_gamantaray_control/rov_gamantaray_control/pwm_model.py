def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalized_to_pwm(
    command: float,
    neutral_us: float,
    min_us: float,
    max_us: float,
) -> float:
    command = clamp(command, -1.0, 1.0)
    if command >= 0.0:
        return neutral_us + command * max(0.0, max_us - neutral_us)
    return neutral_us + command * max(0.0, neutral_us - min_us)


def pwm_to_normalized(
    pwm_us: float,
    neutral_us: float,
    min_us: float,
    max_us: float,
    deadband_us: float,
) -> float:
    pwm_us = clamp(pwm_us, min_us, max_us)
    delta = pwm_us - neutral_us
    deadband_us = max(0.0, deadband_us)
    if abs(delta) <= deadband_us:
        return 0.0
    if delta > 0.0:
        span = max(1e-6, max_us - neutral_us - deadband_us)
        return clamp((delta - deadband_us) / span, 0.0, 1.0)
    span = max(1e-6, neutral_us - min_us - deadband_us)
    return -clamp((-delta - deadband_us) / span, 0.0, 1.0)


def pwm_to_thrust(
    pwm_us: float,
    max_thrust_n: float,
    neutral_us: float,
    min_us: float,
    max_us: float,
    deadband_us: float,
) -> float:
    normalized = pwm_to_normalized(pwm_us, neutral_us, min_us, max_us, deadband_us)
    return max_thrust_n * normalized * abs(normalized)
