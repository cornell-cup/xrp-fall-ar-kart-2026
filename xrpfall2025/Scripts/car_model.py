"""
car_model.py

The kart's dynamics model: the one place it's written down. Both use it:
    - ackermann_simple_ekf.predict()  (the EKF's prediction step)
    - Scripts/twin_replay.py           (the digital twin: the recorded inputs
                                        run through this model on their own)
so whatever you change here, and whatever the twin's tuning shows, carries
straight over to the EKF.

State  [x, y, theta, v]
    x, y   position of the middle of the rear axle, metres
    theta  heading, radians, counter-clockwise
    v      forward speed, m/s
Input
    u      throttle command, -1..1 (the XRP turns it into a wheel speed)
    gamma  steering servo angle, radians (pi/2 = 90 deg = straight ahead)

Kinematic bicycle model, with the speed following the throttle:
    x'     = v cos(theta)
    y'     = v sin(theta)
    theta' = v k                       k = steer_sign tan(steer_ratio (gamma - 90 deg)) / L
    v'     = -beta v + alpha u         steady speed = alpha / beta * u

Parameters (a dict, see params()):
    L            wheelbase, m
    steer_ratio  front-wheel angle per servo angle (the steering linkage)
    steer_sign   +1 if a servo angle below 90 deg turns the kart clockwise, -1 if counter-clockwise
    beta         how quickly the speed follows the throttle, 1/s
    alpha        throttle -> acceleration, (m/s^2) per unit throttle; alpha / beta = top speed
"""

import math

# steer_ratio * (gamma - 90 deg) is cut off short of +-90 deg, where tan() blows up
MAX_STEER_RAD = math.radians(80)


def params(L, steer_ratio, steer_sign, beta, alpha):
    return {"L": float(L), "steer_ratio": float(steer_ratio), "steer_sign": float(steer_sign),
            "beta": float(beta), "alpha": float(alpha)}


def turn_per_metre(gamma, P):
    """theta' / v for servo angle gamma (radians)."""
    steer = P["steer_ratio"] * (gamma - math.pi / 2)
    steer = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, steer))
    return P["steer_sign"] * math.tan(steer) / P["L"]


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def step(x, y, th, v, u, k, P, dt):
    """One Euler step of length dt, with throttle u and turn per metre k
    (turn_per_metre(gamma, P)). Returns the new (x, y, theta, v)."""
    c, s = math.cos(th), math.sin(th)
    return (x + v * c * dt,
            y + v * s * dt,
            wrap_pi(th + v * k * dt),
            v + (-P["beta"] * v + P["alpha"] * u) * dt)


def jacobian(th, v, k, P, dt):
    """d(new state) / d(state) of step(), at the state before it, as rows."""
    c, s = math.cos(th), math.sin(th)
    return [[1.0, 0.0, -v * s * dt, c * dt],
            [0.0, 1.0,  v * c * dt, s * dt],
            [0.0, 0.0,  1.0,        k * dt],
            [0.0, 0.0,  0.0,        1.0 - P["beta"] * dt]]
