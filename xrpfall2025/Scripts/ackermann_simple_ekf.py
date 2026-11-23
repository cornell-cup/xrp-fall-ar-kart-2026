import math
import time

import numpy as np

import car_model


class ackermann_simple_ekf:
    """
    EKF for the Ackermann (car-like) kart.

    State  [x, y, theta, v]^T
        x, y   position in metres
        theta  heading in radians, counter-clockwise
        v      forward speed in m/s
    Input  [u, gamma]^T
        u      throttle command, -1..1 (the XRP turns it into a wheel speed)
        gamma  steering servo angle in radians (pi/2 = 90° = straight ahead)

    Motion model (kinematic bicycle, with the speed following the throttle;
    the equations themselves are in car_model.py, shared with the digital twin):
        x'     = v cos(theta)
        y'     = v sin(theta)
        theta' = s v tan(r (gamma - 90°)) / L
               s = steer_sign: +1 if a servo angle below 90° turns the kart
               clockwise, -1 if it turns it counter-clockwise
               r = steer_ratio: how far the front wheels turn per degree of
               servo (the steering linkage), 1 = as far as the servo
        v'     = -beta v + alpha u        (steady speed = alpha / beta * u)

    Process noise: the position and heading noise grow with distance driven,
    not with time: they're scaled by |v| / top speed (top speed = alpha / beta,
    the steady speed at full throttle), so a parked kart doesn't grow less
    certain of where it is. The speed noise grows with time, so the filter
    still allows for the kart starting off.

    predict() runs the model forward to the current time. Each sensor has its
    own correction, called whenever that sensor has a new reading:
        encoder_correction()  speed (and optionally turn rate) from the rear wheel encoders
        imu_correction()      heading from the IMU
        camera_correction()   x, y, heading from an AprilTag
    Each returns the new (x, y, theta, v) as floats. get_last_nis() then gives
    that correction's normalized innovation squared, for checking the tuning.
    """

    # predict() integrates in steps no longer than this, and catches up at most
    # MAX_PREDICT_S at once (e.g. after a pause in the packets)
    PREDICT_STEP_S = 0.02
    MAX_PREDICT_S = 1.0
    # gamma - 90° is cut off short of ±90°, where tan() would blow up
    MAX_STEER_RAD = car_model.MAX_STEER_RAD

    def __init__(self, wheel_radius, distance_between_wheels, L,
                 model_position_uncertainty=0.01, model_heading_uncertainty=0.05,
                 model_velocity_uncertainty=0.25,
                 sensor_uncertainty=6.5e-5,
                 encoder_speed_uncertainty=0.25, encoder_yaw_rate_uncertainty=3.0,
                 use_encoder_yaw_rate=True,
                 camera_position_uncertainty=1e-4, camera_heading_uncertainty=3e-3,
                 initial_x=0.0, initial_y=0.0, initial_theta=math.pi / 2, initial_v=0.0,
                 initial_pwm=0.0, initial_servo_angle=math.pi / 2,
                 beta=10.0, alpha=3.5, lpf_alpha=0.025, steer_sign=1.0, steer_ratio=1.0,
                 speed_scaled_noise=True):
        """
        wheel_radius, distance_between_wheels (rear track), L (wheelbase): metres
        model_*_uncertainty: process noise, as variance per second (m², rad², (m/s)²).
            With speed_scaled_noise, the position and heading ones are per
            second at top speed (alpha / beta), less when slower, none when stopped
        sensor_uncertainty: IMU heading variance (rad²)
        encoder_speed_uncertainty: encoder speed variance ((m/s)²)
        encoder_yaw_rate_uncertainty: encoder turn-rate variance ((rad/s)²)
        use_encoder_yaw_rate: also correct from the difference between the two
            rear wheels. Only meaningful if they can turn at different speeds
        camera_position_uncertainty, camera_heading_uncertainty: AprilTag pose
            variance (m² for x and y, rad² for heading): the camera noise Qc
        initial_theta: starting heading in radians; pi/2 (90°) = the way the
            kart faces on the start line, as in fusion's map frame
        initial_pwm, initial_servo_angle: the starting input (throttle -1..1,
            servo angle in radians, pi/2 = straight)
        beta: how quickly the speed follows the throttle (1/s)
        alpha: throttle -> acceleration gain; alpha / beta is the top speed
        lpf_alpha: weight of the new value in low_pass_filter()
        steer_sign: +1 if a servo angle below 90° turns the kart clockwise,
            -1 if it turns it counter-clockwise
        steer_ratio: front-wheel angle per servo angle (from 90°), e.g. from the
            measured turning circle: atan(L / radius) / servo angle at full lock
        speed_scaled_noise: scale the position and heading process noise by
            |v| / top speed (see above). False = per second whatever the speed
        """
        # System characteristics
        self._wheel_radius = wheel_radius
        self._distance_between_wheels = distance_between_wheels
        self._length_of_car = L
        self._steer_sign = steer_sign
        self._steer_ratio = steer_ratio
        self._damping = beta
        self._pwm_v_mapping = alpha
        self._lpf_alpha = lpf_alpha

        self._P = np.diag([1e-2, 1e-2, 1e-2, 1e-4])

        # State and input (column vectors)
        self._state = np.array([[float(initial_x)],
                                [float(initial_y)],
                                [float(initial_theta)],
                                [float(initial_v)]])
        self._input = np.array([[float(initial_pwm)],
                                [float(initial_servo_angle)]])

        # Prediction: process noise per second, scaled by each step's length.
        # The position and heading part (_R_motion) is also scaled by speed
        self._R = np.diag([model_position_uncertainty, model_position_uncertainty,
                           model_heading_uncertainty, model_velocity_uncertainty])
        self._R_motion = np.diag([model_position_uncertainty, model_position_uncertainty,
                                  model_heading_uncertainty, 0.0])
        self._R_time = np.diag([0.0, 0.0, 0.0, model_velocity_uncertainty])
        self._speed_scaled_noise = speed_scaled_noise
        # top speed: the steady speed at full throttle
        self._top_speed = alpha / beta if beta > 0 and alpha > 0 else None
        self._lastPredictedTime = None

        # The motion model's parameters, for car_model
        self._model = car_model.params(L, steer_ratio, steer_sign, beta, alpha)

        # Correction
        self._I = np.identity(4)
        self._Qi = np.array([[sensor_uncertainty]])
        self._q_speed = encoder_speed_uncertainty
        self._q_yaw_rate = encoder_yaw_rate_uncertainty
        self._use_encoder_yaw_rate = use_encoder_yaw_rate
        self._Qc = np.diag([camera_position_uncertainty, camera_position_uncertainty,
                            camera_heading_uncertainty])
        self._last_nis = None
        self._last_nis_dof = 0

    # -------- Helpers --------
    @staticmethod
    def wrap_pi(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def get_uncertainty(self):
        return self._P

    def get_state(self):
        return self._state

    def get_last_nis(self):
        """(NIS, degrees of freedom) of the last correction: y^T S^-1 y, where y
        is the innovation and S its predicted covariance. If the filter's noise
        values are right, NIS averages to the degrees of freedom (the number of
        values measured). (None, 0) before the first correction."""
        return self._last_nis, self._last_nis_dof

    def _pose(self):
        s = self._state[:, 0]
        return float(s[0]), float(s[1]), float(s[2]), float(s[3])

    def set_input(self, pwm, servo_angle):
        """The input from now until the next set_input(): throttle -1..1 and
        steering servo angle in radians (pi/2 = straight)."""
        self._input = np.array([[float(pwm)], [float(servo_angle)]])

    def _turn_per_metre(self):
        """theta' / v = steer_sign * tan(steer_ratio * (gamma - 90°)) / L for the
        current servo angle gamma."""
        return car_model.turn_per_metre(self._input[1, 0], self._model)

    def _nis(self, z, h, H, Q, angle_rows=()):
        """NIS this measurement would have, without applying it."""
        y = z - h
        for row in angle_rows:
            y[row, 0] = self.wrap_pi(y[row, 0])
        S = H @ self._P @ H.T + Q
        return float((y.T @ np.linalg.inv(S) @ y)[0, 0])

    def _update(self, z, h, H, Q, angle_rows=()):
        """Standard EKF correction. Rows listed in angle_rows are headings,
        whose difference is wrapped to -pi..pi."""
        y = z - h
        for row in angle_rows:
            y[row, 0] = self.wrap_pi(y[row, 0])
        S = H @ self._P @ H.T + Q
        S_inv = np.linalg.inv(S)
        self._last_nis = float((y.T @ S_inv @ y)[0, 0])
        self._last_nis_dof = y.shape[0]
        K = self._P @ H.T @ S_inv
        self._state = self._state + K @ y
        self._state[2, 0] = self.wrap_pi(self._state[2, 0])
        self._P = (self._I - K @ H) @ self._P
        self._P = (self._P + self._P.T) / 2   # keep it symmetric against rounding
        return self._pose()

    # -------- Prediction --------
    def predict(self, now=None):
        """Runs the motion model forward from the last predict() to now
        (time.monotonic() seconds if not given)."""
        if now is None:
            now = time.monotonic()
        if self._lastPredictedTime is None:
            self._lastPredictedTime = now
            return self._pose()
        total = now - self._lastPredictedTime
        self._lastPredictedTime = now
        if total <= 0:
            return self._pose()
        total = min(total, self.MAX_PREDICT_S)

        u = self._input[0, 0]
        k = self._turn_per_metre()
        steps = max(1, int(math.ceil(total / self.PREDICT_STEP_S)))
        dt = total / steps
        for _ in range(steps):
            x, y, th, v = (float(a) for a in self._state[:, 0])
            # the model step (car_model.py), and its Jacobian at the state before it
            self._state = np.array(car_model.step(x, y, th, v, u, k, self._model, dt)).reshape(4, 1)
            G = np.array(car_model.jacobian(th, v, k, self._model, dt))
            if self._speed_scaled_noise and self._top_speed:
                Rk = self._R_motion * (abs(v) / self._top_speed) + self._R_time
            else:
                Rk = self._R
            self._P = G @ self._P @ G.T + Rk * dt
        return self._pose()

    # -------- Corrections --------
    def encoder_correction(self, delta_left, delta_right, dt):
        """delta_left / delta_right: how far each rear wheel turned (revolutions)
        over the last dt seconds."""
        if dt <= 0:
            return self._pose()
        d_left = delta_left * 2 * math.pi * self._wheel_radius
        d_right = delta_right * 2 * math.pi * self._wheel_radius
        v_e = (d_left + d_right) / 2 / dt
        v = self._state[3, 0]

        if self._use_encoder_yaw_rate:
            k = self._turn_per_metre()
            w_e = (d_right - d_left) / self._distance_between_wheels / dt
            z = np.array([[v_e], [w_e]])
            h = np.array([[v], [v * k]])
            H = np.array([[0, 0, 0, 1],
                          [0, 0, 0, k]])
            Q = np.diag([self._q_speed, self._q_yaw_rate])
        else:
            z = np.array([[v_e]])
            h = np.array([[v]])
            H = np.array([[0, 0, 0, 1]])
            Q = np.array([[self._q_speed]])
        return self._update(z, h, H, Q)

    def imu_correction(self, yaw):
        """yaw: heading in radians, in the same frame as the state."""
        z = np.array([[yaw]])
        h = np.array([[self._state[2, 0]]])
        H = np.array([[0, 0, 1, 0]])
        return self._update(z, h, H, self._Qi, angle_rows=(0,))

    _Hc = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0]])
    # (the camera noise _Qc is set in __init__ from camera_*_uncertainty)

    def camera_correction(self, xc, yc, tc, noise_scale=1.0):
        """noise_scale multiplies the camera noise (Qc), e.g. to trust a far
        tag less."""
        z = np.array([[xc], [yc], [tc]])
        h = self._state[0:3].copy()
        return self._update(z, h, self._Hc, self._Qc * noise_scale, angle_rows=(2,))

    def camera_nis(self, xc, yc, tc, noise_scale=1.0):
        """NIS camera_correction(xc, yc, tc, noise_scale) would have, without
        applying it (for rejecting measurements that can't be right)."""
        z = np.array([[xc], [yc], [tc]])
        return self._nis(z, self._state[0:3].copy(), self._Hc, self._Qc * noise_scale, angle_rows=(2,))

    # -------- Direct adjustments --------
    def shift(self, dx, dy):
        self._state[0, 0] += dx
        self._state[1, 0] += dy
        return float(self._state[0, 0]), float(self._state[1, 0]), float(self._state[2, 0])

    def shift_heading(self, dtheta):
        """Turns the heading by dtheta (radians)."""
        self._state[2, 0] = self.wrap_pi(self._state[2, 0] + dtheta)
        return float(self._state[2, 0])

    def low_pass_filter(self, x, y, theta):
        a = self._lpf_alpha
        self._state[0, 0] = self._state[0, 0] * (1 - a) + x * a
        self._state[1, 0] = self._state[1, 0] * (1 - a) + y * a
        # heading moved along the short way round
        self._state[2, 0] = self.wrap_pi(
            self._state[2, 0] + a * self.wrap_pi(theta - self._state[2, 0]))