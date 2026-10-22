#!/usr/bin/env python3
"""Pure PX4 RC signal validation; throttle/stick POSITION is informational only."""
import math

class RcGateError(ValueError):
    pass

def validated_channel(channel, rc_msg):
    if not isinstance(channel, int) or not 1 <= channel <= 18:
        raise RcGateError('PX4 RC mapping must be between 1 and 18')
    if rc_msg is None or len(rc_msg.channels) < channel:
        raise RcGateError('RC channel not in fresh live MAVROS RCIn message')
    pwm = float(rc_msg.channels[channel - 1])
    if not math.isfinite(pwm) or not 750 <= pwm <= 2250:
        raise RcGateError('RC raw PWM outside reasonable bounds')
    return pwm

def throttle_fraction(pwm, low, high, rev):
    values = (pwm, low, high, rev)
    if not all(math.isfinite(float(x)) for x in values):
        raise RcGateError('Non-finite RC configuration or PWM')
    if not 800 <= low < high <= 2200 or high - low < 500:
        raise RcGateError('RC throttle endpoint calibration invalid')
    if rev not in (-1, 1):
        raise RcGateError('RC throttle reverse flag invalid')
    if not low - 70 <= pwm <= high + 70:
        raise RcGateError('RC throttle PWM outside calibrated endpoints')
    unit = min(1.0, max(0.0, (pwm - low) / (high - low)))
    return 1.0 - unit if rev == -1 else unit

def neutral_stick(pwm, low, trim, high):
    if not all(math.isfinite(float(x)) for x in (pwm, low, trim, high)):
        return False
    if not (800 <= low < trim < high <= 2200 and high - low >= 500):
        return False
    # Calibrated center +/- 7% full stick span (pilot should release X/Y/yaw sticks).
    return abs(pwm - trim) <= .07 * (high - low)

def is_handoff_ready(rc, cal):
    """Validate real RC data/calibration, NOT any throttle or centering position.

    No commands are emitted. The pilot must lower real throttle after POSCTL
    handoff to descend; mid throttle can hold and high throttle can climb.
    """
    if rc is None:
        return False, 'no recent physical RC sample'
    try:
        th = cal['throttle']
        frac = throttle_fraction(validated_channel(th['ch'], rc), th['min'], th['max'], th['rev'])
        centered=[]
        for name in ('roll','pitch','yaw'):
            ax=cal[name]
            pwm=validated_channel(ax['ch'],rc)
            if not (800 <= ax['min'] < ax['trim'] < ax['max'] <= 2200 and
                    ax['max']-ax['min'] >= 500 and
                    ax['min']-70 <= pwm <= ax['max']+70):
                raise RcGateError(name+' RC calibration/PWM invalid')
            centered.append(neutral_stick(pwm,ax['min'],ax['trim'],ax['max']))
        return True, 'throttle=%.1f%% neutral_roll_pitch_yaw=%s (stick POSITION is NOT a gate)' % (
            100*frac, all(centered))
    except (RcGateError,KeyError,TypeError,ValueError,AttributeError) as exc:
        return False,str(exc)
