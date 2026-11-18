# import time, math, network, socket, json
import time, math, socket, json
import numpy as np
# from XRPLib.defaults import *

class ackermann_simple_ekf:
    """
    EKF: state [x, y, theta]. How laptop_fusion.py uses it with the XRP's separate
    sensor packets (xrp_raw_sensors.py sends ENC and IMU packets on their own):
        each ENC packet  -> predict(dl, dr): dl / dr = how far each rear wheel
                            turned (revolutions) since the previous ENC packet.
                            The XRP sends running totals; fusion takes the
                            difference (so a lost packet loses no distance)
        each IMU packet  -> correct(heading): the IMU heading (radians, map frame)
        each tag reading -> camera_correction(x, y, theta)
    get_last_nis() / camera_nis() only report how well each measurement agreed
    with the filter (for pose_overlay.py's NIS table); they don't change the estimate.
    """
    def __init__(self, wheel_radius, distance_between_wheels, initial_position_uncertainty, initial_heading_uncertainty, sensor_uncertainty, initial_left_encoder_value, initial_right_encoder_value, initial_x, initial_y, initial_theta):

        # System Characteristics
        self._wheel_radius = wheel_radius
        self._alpha = 0.025
        self._distance_between_wheels = distance_between_wheels
        self._R = np.array([[initial_position_uncertainty, 0, 0], 
                            [0, initial_position_uncertainty, 0],
                            [0, 0, initial_heading_uncertainty]])
        self._F = np.array([[1.0, 0.0, 1.0],
                            [0.0, 1.0, 1.0],
                            [0.0, 0.0, 1.0]])
        self._Q = sensor_uncertainty
        self._C = np.array([0, 0, 1])
        self._Epsilon = np.array([[1e-3, 0.0,  0.0],
                                  [0.0,  1e-3, 0.0],
                                  [0.0,  0.0,  1e-2]])

        self._I = np.identity(3)
        # self._K_gain = [0.0, 0.0, 0.0]

        # State
        self._state = np.array([[initial_x],
                                [initial_y],
                                [initial_theta]])

        # Input
        self._l = initial_left_encoder_value
        self._r = initial_right_encoder_value

        # NIS of the last correction (reporting only)
        self._last_nis = None
        self._last_nis_dof = 0

    # -------- Helpers --------
    def wrap_pi(self, a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi
        
    def get_uncertainty(self):
        return self._Epsilon

    def get_last_nis(self):
        """(NIS, degrees of freedom) of the last correct() / camera_correction():
        innovation^T S^-1 innovation. Reporting only. (None, 0) before the first."""
        return self._last_nis, self._last_nis_dof

    def predict(self, left_encoder, right_encoder):
        """left_encoder / right_encoder: how far each wheel turned (revolutions)
        since the last call, i.e. since the previous ENC packet."""
        # dl = left_encoder - self._l
        # dr = right_encoder - self._r
        # self._l, self._r = left_encoder, right_encoder

        dl = left_encoder
        dr = right_encoder

        average = np.abs(0.5 * (dl + dr))

        
            
        ds_l = dl * 2.0 * math.pi * self._wheel_radius
        ds_r = dr * 2.0 * math.pi * self._wheel_radius
        ds = 0.5 * (ds_l + ds_r)
        dtheta_enc = (ds_r - ds_l) / self._distance_between_wheels

        # error increases with larger encoder values, but always about 5% of true distance traveled
        self._R[0][0] = ds*0.1
        self._R[1][1] = ds*0.1
        self._R[2][2] = ds*0.1
    
        # ---- Prediction ----
        th_mid = self.wrap_pi(self._state[2][0] + 0.5 * dtheta_enc)

        self._state[0][0] = self._state[0][0] + (ds * math.cos(th_mid))
        self._state[1][0] = self._state[1][0] + (ds * math.sin(th_mid))
        self._state[2][0] = self.wrap_pi(self._state[2][0] + dtheta_enc)

        self._F[0][2] = -ds * math.cos(th_mid)
        self._F[1][2] = ds * math.sin(th_mid)

        self._Epsilon = self._F@self._Epsilon@(self._F.T) + self._R

        return self._state[0][0], self._state[1][0], self._state[2][0]

    def correct(self, measurement):    
        z_matrix = np.array([[0],
                             [0],
                             [measurement]])

        imu_Q = np.array([[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, (((1)*np.pi)/180)**2]]) # (degree*pi)/180 --> radians
        
        imu_C = np.array([[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 1]])
        
        imu_C_transposed = imu_C.T

        non_inverted_value = imu_C@self._Epsilon@imu_C_transposed + imu_Q

        inverse = 1/non_inverted_value[2][2]
        
        # inverse = np.linalg.inv(imu_C@self._Epsilon@imu_C_transposed + imu_Q)

        imu_K = (self._Epsilon@imu_C_transposed)*inverse
        
        innovation = z_matrix - imu_C@self._state
        # the short way round, so 179° and -179° are 2° apart rather than 358°
        innovation[2][0] = self.wrap_pi(innovation[2][0])
        # (reporting only)
        self._last_nis = float(innovation[2][0] ** 2 * inverse)
        self._last_nis_dof = 1
        self._state = self._state + imu_K@innovation

        self._Epsilon = (self._I - imu_K@imu_C)@self._Epsilon
        
        self._state[2][0] = self.wrap_pi(self._state[2][0])

        return self._state[0][0], self._state[1][0], self._state[2][0]
    
    def camera_correction(self, AprilTag_x, AprilTag_y, AprilTag_th):
        new_state = np.array([[0], [0], [0]])
        z_matrix = np.array([[AprilTag_x],
                             [AprilTag_y],
                             [AprilTag_th]])

        camera_Q = np.array([[(0.02)**2, 0, 0], 
                             [0, (0.02)**2, 0],
                             [0, 0, (((3)*np.pi)/180)**2]])
        
        camera_C = np.array([[1, 0, 0],
                             [0, 1, 0],
                             [0, 0, 1]])
        
        camera_C_transposed = camera_C.T
        
        inverse = np.linalg.inv(camera_C@self._Epsilon@camera_C_transposed + camera_Q)

        # (reporting only)
        innovation = z_matrix - camera_C@self._state
        self._last_nis = float((innovation.T@inverse@innovation)[0][0])
        self._last_nis_dof = 3

        camera_K = self._Epsilon@camera_C_transposed@inverse
        
        new_state = self._state + camera_K@(z_matrix - camera_C@self._state)

        self._Epsilon = (self._I - camera_K@camera_C)@self._Epsilon
    
        new_state[2][0] = self.wrap_pi(new_state[2][0])

        self.low_pass_filter(new_state[0][0], new_state[1][0], new_state[2][0])
        return self._state[0][0], self._state[1][0], self._state[2][0]

    def camera_nis(self, AprilTag_x, AprilTag_y, AprilTag_th):
        """NIS camera_correction() would report for this reading, without
        applying it (same matrices as camera_correction)."""
        z_matrix = np.array([[AprilTag_x], [AprilTag_y], [AprilTag_th]])
        camera_Q = np.array([[1e-4, 0, 0],
                             [0, 1e-4, 0],
                             [0, 0, 3e-3]])
        camera_C = np.identity(3)
        inverse = np.linalg.inv(camera_C@self._Epsilon@camera_C.T + camera_Q)
        innovation = z_matrix - camera_C@self._state
        return float((innovation.T@inverse@innovation)[0][0])

    def shift(self, dx, dy):
        self._state[0][0] = self._state[0][0] + dx
        self._state[1][0] = self._state[1][0] + dy
        return self._state[0][0], self._state[1][0], self._state[2][0]

    def low_pass_filter(self, x, y, theta):
        self._state[0][0] = self._state[0][0] * (1 - self._alpha) + x * self._alpha
        self._state[1][0] = self._state[1][0] * (1 - self._alpha) + y * self._alpha
        self._state[2][0] = self._state[2][0] * (1 - self._alpha) + theta * self._alpha