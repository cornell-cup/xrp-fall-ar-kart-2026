"""
fusion_core.py

The sensor fusion of laptop_fusion.py, one packet at a time: everything its
sensor thread does with an XRP packet (which XRP to follow, packet numbers,
gyro scale and drift, the IMU heading offset, the EKF's predict and
corrections, and working in the AprilTag poses). It's a class that's given each
packet with the time it arrived, instead of reading the clock and a socket
itself, so the very same code runs:
    - live, in laptop_fusion.py's sensor thread
    - offline, in Scripts/twin_replay.py, re-running a recording with
      different settings to tune the EKF

The settings are a dict with the names of laptop_fusion.py's constants
(CONFIG_KEYS); laptop_fusion.py builds it from its own constants, so those
stay where they are. DEFAULT_CONFIG is only a fallback for recordings made
without one.
"""

import math
import statistics

import numpy as np

import car_model
from ackermann_simple_ekf import ackermann_simple_ekf

CONFIG_KEYS = [
    # kart and model
    "WHEEL_RADIUS", "CAR_WIDTH", "WHEELBASE", "STEER_RATIO", "STEER_SIGN",
    "XRP_MAX_RPM", "SPEED_RESPONSE",
    "TAG_POINT_AHEAD_OF_REAR_AXLE_M", "OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M",
    # EKF noise
    "MODEL_POS_VAR", "MODEL_HEADING_VAR", "MODEL_SPEED_VAR",
    "ENCODER_SPEED_VAR", "ENCODER_YAW_RATE_VAR", "IMU_HEADING_VAR",
    "CAMERA_POS_VAR", "CAMERA_HEADING_VAR", "ENCODER_YAW_RATE",
    # tags
    "APPLY_CAMERA_CORRECTION", "TAG_FRESHNESS_S", "CAMERA_LATENCY_S", "POSE_HISTORY_S",
    "TAG_HEADING_GATE_DEG", "TAG_HEADING_WINDOW_S", "TAG_HEADING_MIN_READINGS",
    "TAG_HEADING_MAX_SPREAD_DEG", "TAG_HEADING_GAIN", "TAG_HEADING_MAX_CONSISTENT_DEG",
    "CAMERA_NIS_GATE", "CAMERA_RESYNC_S", "MAX_TAG_DISTANCE_M", "TAG_NOISE_REF_M",
    # IMU and packets
    "GYRO_SCALE", "STILL_AFTER_S", "XRP_LEASE_S", "XRP_TICKS_PERIOD_MS",
]

# laptop_fusion.py's values when this was written (fallback only)
DEFAULT_CONFIG = {
    "WHEEL_RADIUS": 0.04176, "CAR_WIDTH": 0.11928, "WHEELBASE": 0.13643,
    "STEER_RATIO": math.degrees(math.atan(0.13643 / (0.5461 / 2))) / 60.0, "STEER_SIGN": 1.0,
    "XRP_MAX_RPM": 150, "SPEED_RESPONSE": 10.0,
    "TAG_POINT_AHEAD_OF_REAR_AXLE_M": 0.03513, "OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M": 0.03513,
    "MODEL_POS_VAR": 1e-3, "MODEL_HEADING_VAR": 0.0075, "MODEL_SPEED_VAR": 0.25,
    "ENCODER_SPEED_VAR": 1e-4, "ENCODER_YAW_RATE_VAR": 3.0, "IMU_HEADING_VAR": 1e-6,
    "CAMERA_POS_VAR": 0.04 ** 2, "CAMERA_HEADING_VAR": math.radians(6) ** 2, "ENCODER_YAW_RATE": False,
    "APPLY_CAMERA_CORRECTION": True, "TAG_FRESHNESS_S": 0.3, "CAMERA_LATENCY_S": 0.27,
    "POSE_HISTORY_S": 1.0, "TAG_HEADING_GATE_DEG": 6.0, "TAG_HEADING_WINDOW_S": 2.0,
    "TAG_HEADING_MIN_READINGS": 8, "TAG_HEADING_MAX_SPREAD_DEG": 4.0, "TAG_HEADING_GAIN": 0.05,
    "TAG_HEADING_MAX_CONSISTENT_DEG": 25.0, "CAMERA_NIS_GATE": 16.27, "CAMERA_RESYNC_S": 1.0,
    "MAX_TAG_DISTANCE_M": 1.0, "TAG_NOISE_REF_M": 0.7,
    "GYRO_SCALE": 1.0, "STILL_AFTER_S": 0.3, "XRP_LEASE_S": 3.0, "XRP_TICKS_PERIOD_MS": 1 << 30,
}


def xrp_max_speed(cfg):
    """Throttle 1.0 as a speed, m/s."""
    return cfg["XRP_MAX_RPM"] / 60.0 * 2 * math.pi * cfg["WHEEL_RADIUS"]


def model_params(cfg):
    """car_model's parameters from the settings (as new_ekf() sets up the EKF)."""
    return car_model.params(L=cfg["WHEELBASE"], steer_ratio=cfg["STEER_RATIO"],
                            steer_sign=cfg["STEER_SIGN"], beta=cfg["SPEED_RESPONSE"],
                            alpha=cfg["SPEED_RESPONSE"] * xrp_max_speed(cfg))


def new_ekf(cfg):
    return ackermann_simple_ekf(
        wheel_radius=cfg["WHEEL_RADIUS"],
        distance_between_wheels=cfg["CAR_WIDTH"],
        L=cfg["WHEELBASE"],
        model_position_uncertainty=cfg["MODEL_POS_VAR"],
        model_heading_uncertainty=cfg["MODEL_HEADING_VAR"],
        model_velocity_uncertainty=cfg["MODEL_SPEED_VAR"],
        sensor_uncertainty=cfg["IMU_HEADING_VAR"],
        encoder_speed_uncertainty=cfg["ENCODER_SPEED_VAR"],
        encoder_yaw_rate_uncertainty=cfg["ENCODER_YAW_RATE_VAR"],
        use_encoder_yaw_rate=cfg["ENCODER_YAW_RATE"],
        camera_position_uncertainty=cfg["CAMERA_POS_VAR"],
        camera_heading_uncertainty=cfg["CAMERA_HEADING_VAR"],
        initial_x=0.0, initial_y=0.0, initial_theta=np.pi/2, initial_v=0.0,
        initial_pwm=0.0, initial_servo_angle=np.pi/2,   # servo at 90° = straight
        beta=cfg["SPEED_RESPONSE"], alpha=cfg["SPEED_RESPONSE"] * xrp_max_speed(cfg),
        steer_sign=cfg["STEER_SIGN"],
        steer_ratio=cfg["STEER_RATIO"],
    )


def parse_packet(text):
    """An XRP packet's text -> (kind, fields) or (kind, None) if it isn't one we use.
        ENC,seq,t_ms,total_l,total_r,throttle,servo_deg   encoder running totals + the inputs
        IMU,seq,t_ms,heading                              IMU heading"""
    parts = text.split(",")
    kind = parts[0]
    try:
        if kind == "ENC" and len(parts) == 7:
            return kind, {"seq": int(parts[1]), "t_ms": int(parts[2]),
                          "total_l": float(parts[3]), "total_r": float(parts[4]),
                          "throttle": float(parts[5]), "servo_deg": float(parts[6])}
        if kind == "IMU" and len(parts) == 4:
            return kind, {"seq": int(parts[1]), "t_ms": int(parts[2]), "heading": float(parts[3])}
    except ValueError:
        return None, None
    return kind, None


class FusionCore:
    """laptop_fusion's sensor fusion. Call process() for each XRP packet with
    the time it arrived and the newest tag pose the detector has published by
    then. The fused state and the sensor values are attributes afterwards."""

    def __init__(self, cfg, log=print):
        self.cfg = dict(DEFAULT_CONFIG)
        self.cfg.update(cfg)
        self.log = log or (lambda *a, **k: None)
        self.ekf = None

        # the results, as laptop_fusion's globals of the same names
        self.x = 0.0
        self.y = 0.0
        self.theta = np.pi / 2
        self.speed = 0.0
        self.enc_l = 0.0
        self.enc_r = 0.0
        self.heading = 0.0
        self.total_turned_raw = 0.0
        self.total_driven_m = 0.0
        self.imu_heading_offset = 0.0
        self.pose_w = None
        self.xrp_ip = None
        self.xrp_last_seen = 0.0
        # NIS of each correction since take_nis(): [(kind, nis, dof), ...]
        self.nis = []
        # extra, for the replay: the last encoder speed measurement and the inputs
        self.last_v_enc = None
        self.throttle = 0.0
        self.servo_deg = 90.0
        self.camera_used = False   # whether the last packet applied a tag

        # the sensor thread's own variables
        self.enc_count = self.imu_count = 0
        self.last_print = None
        self.last_ignored_print = 0.0
        self.warned_old_xrp = False
        self.last_enc_seq = None
        self.last_enc_seq_time = 0.0
        self.last_total_l = self.last_total_r = 0.0
        self.last_enc_ms = 0
        self.last_imu_seq = None
        self.last_imu_seq_time = 0.0
        self.lost_enc = self.lost_imu = 0
        self.pose_history = []
        self.last_tag_time = 0.0
        self.recent_heading_diffs = []
        self.last_scale_src = None
        self.scaled_heading = 0.0
        self.cam_rejected_since = None
        self.last_facing_warning = 0.0
        self.tag_stats = {}
        self.moved_time = None
        self.last_raw_heading = None
        self.heading_offset_set = False

    # -------- results --------
    def take_nis(self):
        out, self.nis = self.nis, []
        return out

    def _record_nis(self, kind, nis=None, dof=None):
        if nis is None:
            nis, dof = self.ekf.get_last_nis()
        if nis is None:
            return
        self.nis.append((kind, round(nis, 4), dof))

    def fused(self):
        """The estimate as laptop_fusion sends it: the output point
        (OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M), heading, speed, and the x, y, theta covariance."""
        d = self.cfg["OUTPUT_POINT_AHEAD_OF_REAR_AXLE_M"]
        return {"x": self.x + d * math.cos(self.theta), "y": self.y + d * math.sin(self.theta),
                "theta": self.theta, "v": self.speed,
                "cov": self.ekf.get_uncertainty()[:3, :3].tolist()}

    # -------- one packet --------
    def process(self, text, ip, now, tag_pose):
        """text: the packet as received (decoded); ip: who sent it; now: when it
        arrived (time.time()); tag_pose: the newest tag pose published by the
        detector by then (laptop_fusion's latest_robot_pose_world), or None.
        Returns the packet's kind ("ENC" / "IMU") if it was used, else None."""
        cfg = self.cfg
        self.camera_used = False
        kind, f = parse_packet(text)
        if f is None:
            if kind in ("RAW", "RAW2") and not self.warned_old_xrp:
                self.log(f"[Sensor] {ip} is sending the old combined {kind} packets; "
                         "upload the new xrp_raw_sensors.py to it (ENC / IMU packets)")
                self.warned_old_xrp = True
            return None
        if self.moved_time is None:
            self.moved_time = now
        if self.last_print is None:
            self.last_print = now

        # ---- Which XRP this laptop follows (first come, first served) ----
        if self.xrp_ip is not None and ip != self.xrp_ip and now - self.xrp_last_seen < cfg["XRP_LEASE_S"]:
            if now - self.last_ignored_print > 5.0:
                self.log(f"[XRP] ignoring {ip} (locked onto {self.xrp_ip})")
                self.last_ignored_print = now
            return None
        if ip != self.xrp_ip:
            if self.xrp_ip is not None:
                self.log(f"[XRP] {self.xrp_ip} went quiet, switching to {ip}")
                # a different robot's encoders and heading: start the filter over
                self.ekf = None
            self.xrp_ip = ip
            self.last_enc_seq = self.last_imu_seq = None
            self.log(f"[XRP] locked onto {self.xrp_ip}")
        self.xrp_last_seen = now

        if self.ekf is None:
            self.ekf = new_ekf(cfg)
            self.heading_offset_set = False
            self.last_raw_heading = None
            self.pose_history = []
            self.log(f"[EKF] Initialized for {ip}")
        ekf = self.ekf
        seq = f["seq"]

        if kind == "ENC":
            # ---- Encoder packet: how far each wheel turned, and over how long ----
            if self.last_enc_seq is not None and seq <= self.last_enc_seq and now - self.last_enc_seq_time < 1.0:
                return None   # arrived late or twice; its totals are already counted
            if self.last_enc_seq is None or seq <= self.last_enc_seq:
                # first packet, or the count started over after a gap: the XRP
                # restarted (which takes several seconds), and its encoders did too.
                # Just take this one as the starting point
                new_enc_l = new_enc_r = 0.0
                enc_dt = 0.0
            else:
                # (after a gap without a restart, e.g. briefly out of Wi-Fi range,
                # the totals still include what was driven meanwhile)
                new_enc_l = f["total_l"] - self.last_total_l
                new_enc_r = f["total_r"] - self.last_total_r
                enc_dt = ((f["t_ms"] - self.last_enc_ms) % cfg["XRP_TICKS_PERIOD_MS"]) / 1000.0
                self.lost_enc += seq - self.last_enc_seq - 1
            self.last_enc_seq, self.last_total_l, self.last_total_r, self.last_enc_ms = \
                seq, f["total_l"], f["total_r"], f["t_ms"]
            self.last_enc_seq_time = now
            self.enc_l, self.enc_r = new_enc_l, new_enc_r
            self.total_driven_m += (self.enc_l + self.enc_r) / 2 * 2 * math.pi * cfg["WHEEL_RADIUS"]
            if self.enc_l != 0.0 or self.enc_r != 0.0:
                self.moved_time = now
            self.enc_count += 1
        else:
            # ---- IMU packet ----
            if self.last_imu_seq is not None and seq <= self.last_imu_seq and now - self.last_imu_seq_time < 1.0:
                return None   # arrived late or twice
            if self.last_imu_seq is not None and seq > self.last_imu_seq:
                self.lost_imu += seq - self.last_imu_seq - 1
            self.last_imu_seq = seq
            self.last_imu_seq_time = now
            # IMU heading with its turns scaled by GYRO_SCALE; the raw turning is
            # also added up for measuring GYRO_SCALE
            new_heading = f["heading"]
            if self.last_scale_src is None:
                self.scaled_heading = new_heading
            else:
                d_raw = (new_heading - self.last_scale_src + math.pi) % (2 * math.pi) - math.pi
                self.scaled_heading = (self.scaled_heading + cfg["GYRO_SCALE"] * d_raw + math.pi) % (2 * math.pi) - math.pi
                self.total_turned_raw += d_raw
            self.last_scale_src = new_heading
            self.heading = self.scaled_heading

            # ---- Gyro drift ----
            if self.last_raw_heading is not None and now - self.moved_time >= cfg["STILL_AFTER_S"]:
                # standing still: hold the heading
                d_heading = (self.heading - self.last_raw_heading + math.pi) % (2 * math.pi) - math.pi
                self.imu_heading_offset = (self.imu_heading_offset - d_heading + math.pi) % (2 * math.pi) - math.pi
            self.last_raw_heading = self.heading

            if not self.heading_offset_set:
                # start facing 90° like the position starts at 0,0, even if the
                # XRP's heading drifted before fusion started
                self.imu_heading_offset = ekf.wrap_pi(np.pi/2 - self.heading)
                self.heading_offset_set = True
                self.log(f"[EKF] Heading set from IMU heading={self.heading:.3f} rad")
            self.imu_count += 1

        # Model forward to now, then the correction for whichever sensor this is
        x, y, theta, speed = ekf.predict(now)
        if kind == "ENC":
            self.last_v_enc = None
            if enc_dt > 0:
                x, y, theta, speed = ekf.encoder_correction(self.enc_l, self.enc_r, enc_dt)
                self._record_nis("encoder")
                self.last_v_enc = (self.enc_l + self.enc_r) / 2 * 2 * math.pi * cfg["WHEEL_RADIUS"] / enc_dt
            # the throttle and steering the XRP is applying from now on
            self.throttle, self.servo_deg = f["throttle"], f["servo_deg"]
            ekf.set_input(f["throttle"], math.radians(f["servo_deg"]))
        else:
            x, y, theta, speed = ekf.imu_correction(ekf.wrap_pi(self.heading + self.imu_heading_offset))
            self._record_nis("IMU")

        pose_w = self.pose_w = tag_pose
        tag_is_fresh = pose_w is not None and (now - pose_w["time"]) < cfg["TAG_FRESHNESS_S"]

        self.pose_history.append([now, x, y, theta])
        while self.pose_history and now - self.pose_history[0][0] > cfg["POSE_HISTORY_S"]:
            self.pose_history.pop(0)

        # A new tag pose: the EKF's camera correction
        if tag_is_fresh and cfg["APPLY_CAMERA_CORRECTION"] and pose_w["time"] != self.last_tag_time:
            x, y, theta, speed = self._tag(pose_w, now, x, y, theta, speed)

        self.x, self.y, self.theta, self.speed = x, y, theta, speed
        if now - self.last_print >= 1.0:
            self._print_status(now)
        return kind

    def _tag(self, pose_w, now, x, y, theta, speed):
        cfg = self.cfg
        ekf = self.ekf
        self.last_tag_time = pose_w["time"]
        tag_dist = math.hypot(pose_w["rel_x"], pose_w["rel_y"])
        st = self.tag_stats.setdefault(int(pose_w["tag_id"]), {
            "n": 0, "used_n": 0, "along": 0.0, "across": 0.0, "heading": 0.0, "dist": 0.0,
            "nis": 0.0, "rejected": 0, "heading_ignored": 0, "heading_consistent": 0, "too_far": 0})
        st["n"] += 1
        st["dist"] += tag_dist
        if tag_dist > cfg["MAX_TAG_DISTANCE_M"]:
            # too far to trust: skipped entirely (no correction, and it doesn't
            # count towards the heading or rejection checks)
            st["too_far"] += 1
            return x, y, theta, speed

        # farther = trusted less
        noise_scale = max(1.0, (tag_dist / cfg["TAG_NOISE_REF_M"]) ** 2)
        taken = pose_w["time"] - cfg["CAMERA_LATENCY_S"]
        then = min(self.pose_history, key=lambda h: abs(h[0] - taken))
        # Position from where the tag was relative to the robot, turned by the
        # estimate's heading, so it doesn't move when the tag's angle flips
        c, s = math.cos(then[3]), math.sin(then[3])
        tag_pos_x = pose_w["tag_x"] - (c * pose_w["rel_x"] - s * pose_w["rel_y"])
        tag_pos_y = pose_w["tag_y"] - (s * pose_w["rel_x"] + c * pose_w["rel_y"])
        # that's the tags' point on the kart; the filter tracks the rear axle
        tag_pos_x -= cfg["TAG_POINT_AHEAD_OF_REAR_AXLE_M"] * c
        tag_pos_y -= cfg["TAG_POINT_AHEAD_OF_REAR_AXLE_M"] * s
        tag_theta = pose_w["theta"]
        heading_diff = ekf.wrap_pi(tag_theta - then[3])
        raw_heading_diff = heading_diff
        heading_ignored = heading_consistent = False
        self.recent_heading_diffs.append([now, raw_heading_diff])
        while self.recent_heading_diffs and now - self.recent_heading_diffs[0][0] > cfg["TAG_HEADING_WINDOW_S"]:
            self.recent_heading_diffs.pop(0)
        gate = math.radians(cfg["TAG_HEADING_GATE_DEG"])
        if abs(heading_diff) > gate:
            diffs = [d for _, d in self.recent_heading_diffs]
            med = statistics.median(diffs)
            spread = statistics.median([abs(d - med) for d in diffs])
            consistent = (len(diffs) >= cfg["TAG_HEADING_MIN_READINGS"] and abs(med) > gate
                          and spread <= math.radians(cfg["TAG_HEADING_MAX_SPREAD_DEG"]))
            if consistent and abs(med) > math.radians(cfg["TAG_HEADING_MAX_CONSISTENT_DEG"]):
                # too big to be drift: probably this tag's facing in TAG_WORLD
                heading_diff = 0.0
                heading_ignored = True
                if now - self.last_facing_warning > 5.0:
                    self.last_facing_warning = now
                    self.log(f"[Tag]    WARNING tag {int(pose_w['tag_id'])}'s heading is a steady "
                             f"{math.degrees(med):+.0f} deg off the estimate: check its gate ends "
                             f"(facing) in laptop_fusion.py. Its headings are being ignored")
            elif consistent:
                # the recent tags agree: the estimate's heading is off
                heading_diff = med
                heading_consistent = True
            else:
                heading_diff = 0.0   # probably the flipped answer: no heading correction
                heading_ignored = True
        # The tag measured the pose at `then`; move it on by however far the
        # filter has moved since, so it's compared with the estimate now
        meas_x = tag_pos_x + (x - then[1])
        meas_y = tag_pos_y + (y - then[2])
        meas_theta = ekf.wrap_pi(theta + heading_diff)

        # How far off the tag says the estimate is, along and across the
        # kart's heading, for the per-tag summary
        ix, iy = meas_x - x, meas_y - y
        c_now, s_now = math.cos(theta), math.sin(theta)
        cam_nis = ekf.camera_nis(meas_x, meas_y, meas_theta, noise_scale)
        self._record_nis("camera", cam_nis, 3)
        rejected = cam_nis > cfg["CAMERA_NIS_GATE"]
        if rejected:
            if self.cam_rejected_since is None:
                self.cam_rejected_since = now
            elif now - self.cam_rejected_since >= cfg["CAMERA_RESYNC_S"]:
                rejected = False   # every tag for a while: trust them
        st["used_n"] += 1
        st["along"] += ix * c_now + iy * s_now
        st["across"] += -ix * s_now + iy * c_now
        st["heading"] += math.degrees(raw_heading_diff)
        st["nis"] += cam_nis
        st["rejected"] += rejected
        st["heading_ignored"] += heading_ignored
        st["heading_consistent"] += heading_consistent
        if not rejected:
            self.cam_rejected_since = None
            try:
                theta_before = theta
                x, y, theta, speed = ekf.camera_correction(meas_x, meas_y, meas_theta, noise_scale)
                self.camera_used = True
                # The IMU's offset: a part of the heading difference that's left
                # (when the tag's heading is used at all, see TAG_HEADING_GAIN)
                if not heading_ignored:
                    theta = ekf.shift_heading(cfg["TAG_HEADING_GAIN"] * ekf.wrap_pi(meas_theta - theta))
                # Heading follows the IMU, whose next correction would undo the
                # change, so the IMU's offset takes all of it (this is what
                # corrects gyro drift)
                self.imu_heading_offset = ekf.wrap_pi(self.imu_heading_offset + ekf.wrap_pi(theta - theta_before))
                # the past moves with it, so the next tag isn't compared with stale history
                last = self.pose_history[-1]
                dx, dy, dth = x - last[1], y - last[2], ekf.wrap_pi(theta - theta_before)
                for h in self.pose_history:
                    h[1] += dx; h[2] += dy; h[3] += dth
                # and the stored heading differences are now that much smaller
                for e in self.recent_heading_diffs:
                    e[1] -= dth
            except Exception as e:
                self.log(f"[EKF] camera_correction error: {e}")
        return x, y, theta, speed

    def _print_status(self, now):
        pose_w = self.pose_w
        if pose_w is None:
            tag_info = "no tag yet"
        else:
            age = now - pose_w["time"]
            if age < self.cfg["TAG_FRESHNESS_S"]:
                tag_info = (f"tag x={pose_w['x']:+.2f} y={pose_w['y']:+.2f} "
                            f"th={math.degrees(pose_w['theta']):+.0f} ({age*1000:.0f}ms)")
            else:
                tag_info = f"tag stale {age:.1f}s"
        self.log(f"[EKF] {self.enc_count} enc + {self.imu_count} imu pkts/s ({self.lost_enc}/{self.lost_imu} lost) | "
                 f"FUSED x={self.x:+.2f} y={self.y:+.2f} th={math.degrees(self.theta):+.0f} v={self.speed:+.2f} | {tag_info}")
        self.enc_count = self.imu_count = 0
        self.lost_enc = self.lost_imu = 0
        self.last_print = now
        # Per tag, on average: where the tag put the kart compared with the
        # estimate (ahead / left of it, and turned). A steady "ahead" while
        # driving (and none while parked) = CAMERA_LATENCY_S is off; an offset
        # for one tag only = that tag's entry in TAG_WORLD is off; offsets that
        # grow with distance = tag size, camera calibration or lens distortion
        for tid in sorted(self.tag_stats):
            st = self.tag_stats[tid]
            n, u = st["n"], st["used_n"]
            line = f"[Tag]    tag {tid}: {n} seen, {st['dist']/n:.2f} m away"
            if st["too_far"]:
                line += f", {st['too_far']} too far (> {self.cfg['MAX_TAG_DISTANCE_M']:g} m)"
            if u:
                line += (f" | puts the kart {st['along']/u*100:+.1f} cm ahead, "
                         f"{st['across']/u*100:+.1f} cm left, {st['heading']/u:+.1f} deg turned "
                         f"left of the estimate | NIS {st['nis']/u:.1f}, {st['rejected']} rejected, "
                         f"{st['heading_ignored']} headings ignored, "
                         f"{st['heading_consistent']} taken from consistent readings")
            self.log(line)
        self.tag_stats.clear()
