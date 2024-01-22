#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math


def madgwick_ahrs_update(gx, gy, gz, ax, ay, az, mx, my, mz, q0, q1, q2, q3, beta, sample_freq):
    s0, s1, s2, s3 = 0.0, 0.0, 0.0, 0.0
    _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3q2, = \
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    q_dot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    q_dot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    q_dot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    q_dot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

    if not (ax == 0.0 and ay == 0.0 and az == 0.0):
        recip_norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= recip_norm
        ay *= recip_norm
        az *= recip_norm

        recip_norm = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
        mx *= recip_norm
        my *= recip_norm
        mz *= recip_norm

        _2q0mx = 2.0 * q0 * mx
        _2q0my = 2.0 * q0 * my
        _2q0mz = 2.0 * q0 * mz
        _2q1mx = 2.0 * q1 * mx
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q0q2 = 2.0 * q0 * q2
        _2q2q3 = 2.0 * q2 * q3
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        # Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q2 * (
                _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (
                     _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (
                     _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

        s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q1 * (
                1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q3 * (
                     _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (
                     _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (
                     _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

        s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q2 * (
                1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (
                     _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (
                     _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (
                     _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

        s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (
                _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (
                     _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (
                     _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

        recip_norm = 1 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)  # Normalise step magnitude
        s0 *= recip_norm
        s1 *= recip_norm
        s2 *= recip_norm
        s3 *= recip_norm

        # Apply feedback step
        q_dot1 -= beta * s0
        q_dot2 -= beta * s1
        q_dot3 -= beta * s2
        q_dot4 -= beta * s3

    # Integrate rate of change of quaternion to yield quaternion
    q0 += q_dot1 * (1.0 / sample_freq)
    q1 += q_dot2 * (1.0 / sample_freq)
    q2 += q_dot3 * (1.0 / sample_freq)
    q3 += q_dot4 * (1.0 / sample_freq)
    recip_norm = 1 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)

    q0 *= recip_norm
    q1 *= recip_norm
    q2 *= recip_norm
    q3 *= recip_norm


if __name__ == '__main__':
    madgwick_ahrs_update(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 100)
