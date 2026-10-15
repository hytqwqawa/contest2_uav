#!/usr/bin/env python3

import math
import os
import rospy

from mavros_msgs.msg import State, RCIn, PlayTuneV2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String


class SystemReadyNode:

    GREEN = "\033[1;32m"
    RED = "\033[1;31m"
    YELLOW = "\033[1;33m"
    CYAN = "\033[1;36m"
    WHITE = "\033[1;37m"
    RESET = "\033[0m"

    def __init__(self):

        # ---------------------------------------------------
        # Parameters
        # ---------------------------------------------------

        self.stable_time = rospy.get_param(
            "~stable_time", 1.5
        )

        self.state_timeout = rospy.get_param(
            "~state_timeout", 2.5
        )

        self.odom_timeout = rospy.get_param(
            "~odom_timeout", 0.30
        )

        self.fast_timeout = rospy.get_param(
            "~fast_timeout", 0.60
        )

        self.battery_timeout = rospy.get_param(
            "~battery_timeout", 2.0
        )

        # Only a conservative initialization gate.
        # Can be adjusted later if desired.
        self.min_battery_percentage = rospy.get_param(
            "~min_battery_percentage", 0.10
        )

        self.min_battery_voltage = rospy.get_param(
            "~min_battery_voltage", 20.0
        )

        # Current verified three-position switch thresholds
        self.low_max = rospy.get_param(
            "~rc_low_max", 1300
        )

        self.high_min = rospy.get_param(
            "~rc_high_min", 1700
        )

        # ---------------------------------------------------
        # State
        # ---------------------------------------------------

        self.px4_state = None
        self.px4_state_time = None

        self.odom_time = None

        self.battery = None
        self.battery_time = None

        self.preflight_ready = False
        self.preflight_time = None

        self.flight_healthy = False
        self.health_time = None

        self.stream_ready = False
        self.stream_time = None

        self.rc_msg = None
        self.rc_time = None

        self.rc_status = ""
        self.rc_status_time = None

        self.offboard_status = ""
        self.offboard_status_time = None

        self.traj_if_status = ""
        self.traj_if_status_time = None

        self.point_status = ""
        self.point_status_time = None

        self.ready_since = None
        self.ready = False
        self.last_ready = False

        self.last_console_time = rospy.Time(0)

        # ---------------------------------------------------
        # Subscribers
        # ---------------------------------------------------

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/mavros/battery",
            BatteryState,
            self.battery_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/mavros/rc/in",
            RCIn,
            self.rc_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/preflight_ready",
            Bool,
            self.preflight_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/offboard_stream_ready",
            Bool,
            self.stream_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/rc_supervisor_status",
            String,
            self.rc_status_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/offboard_manager_status",
            String,
            self.offboard_status_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/trajectory_interface_status",
            String,
            self.traj_if_status_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            self.point_status_cb,
            queue_size=10
        )

        # ---------------------------------------------------
        # Outputs
        # ---------------------------------------------------

        self.ready_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/system_ready",
            Bool,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/system_ready_status",
            String,
            queue_size=1,
            latch=True
        )

        # Optional. Failure to make sound NEVER blocks READY.
        self.tune_pub = rospy.Publisher(
            "/contest2/mavros/play_tune",
            PlayTuneV2,
            queue_size=1
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.1),
            self.timer_cb
        )

    # =======================================================
    # Callbacks
    # =======================================================

    def state_cb(self, msg):
        self.px4_state = msg
        self.px4_state_time = rospy.Time.now()

    def odom_cb(self, msg):
        self.odom_time = rospy.Time.now()

    def battery_cb(self, msg):
        self.battery = msg
        self.battery_time = rospy.Time.now()

    def rc_cb(self, msg):
        self.rc_msg = msg
        self.rc_time = rospy.Time.now()

    def preflight_cb(self, msg):
        self.preflight_ready = msg.data
        self.preflight_time = rospy.Time.now()

    def health_cb(self, msg):
        self.flight_healthy = msg.data
        self.health_time = rospy.Time.now()

    def stream_cb(self, msg):
        self.stream_ready = msg.data
        self.stream_time = rospy.Time.now()

    def rc_status_cb(self, msg):
        self.rc_status = msg.data
        self.rc_status_time = rospy.Time.now()

    def offboard_status_cb(self, msg):
        self.offboard_status = msg.data
        self.offboard_status_time = rospy.Time.now()

    def traj_if_status_cb(self, msg):
        self.traj_if_status = msg.data
        self.traj_if_status_time = rospy.Time.now()

    def point_status_cb(self, msg):
        self.point_status = msg.data
        self.point_status_time = rospy.Time.now()

    # =======================================================
    # Helpers
    # =======================================================

    @staticmethod
    def fresh(stamp, timeout):

        if stamp is None:
            return False

        return (
            rospy.Time.now() - stamp
        ).to_sec() < timeout

    def rc_ch6_ch7(self):

        if (
            self.rc_msg is None or
            len(self.rc_msg.channels) < 7
        ):
            return None, None

        return (
            self.rc_msg.channels[5],
            self.rc_msg.channels[6]
        )

    def battery_ok(self):

        if not self.fresh(
            self.battery_time,
            self.battery_timeout
        ):
            return False

        if self.battery is None:
            return False

        v = self.battery.voltage
        p = self.battery.percentage

        voltage_valid = (
            math.isfinite(v) and
            v >= self.min_battery_voltage
        )

        percentage_valid = (
            math.isfinite(p) and
            p >= self.min_battery_percentage
        )

        return voltage_valid and percentage_valid

    def evaluate(self):

        checks = {}

        checks["PX4 LINK"] = (
            self.fresh(
                self.px4_state_time,
                self.state_timeout
            )
            and self.px4_state is not None
            and self.px4_state.connected
        )

        checks["DISARMED"] = (
            self.px4_state is not None
            and not self.px4_state.armed
        )

        checks["POSCTL"] = (
            self.px4_state is not None
            and self.px4_state.mode == "POSCTL"
        )

        checks["ODOM"] = self.fresh(
            self.odom_time,
            self.odom_timeout
        )

        checks["PREFLIGHT"] = (
            self.fresh(
                self.preflight_time,
                self.fast_timeout
            )
            and self.preflight_ready
        )

        checks["FLIGHT HEALTH"] = (
            self.fresh(
                self.health_time,
                self.fast_timeout
            )
            and self.flight_healthy
        )

        checks["SETPOINT STREAM"] = (
            self.fresh(
                self.stream_time,
                self.fast_timeout
            )
            and self.stream_ready
        )


        checks["RC SUPERVISOR"] = self.fresh(
            self.rc_status_time,
            self.fast_timeout
        )

        checks["OFFBOARD MANAGER"] = self.fresh(
            self.offboard_status_time,
            self.fast_timeout
        )

        checks["TRAJECTORY IF"] = self.fresh(
            self.traj_if_status_time,
            self.fast_timeout
        )


        rc_fresh = self.fresh(
            self.rc_time,
            self.fast_timeout
        )

        ch6, ch7 = self.rc_ch6_ch7()

        checks["CH6 POSCTL"] = (
            rc_fresh
            and ch6 is not None
            and ch6 >= self.high_min
        )

        checks["CH7 SAFE"] = (
            rc_fresh
            and ch7 is not None
            and ch7 <= self.low_max
        )

        return checks

    # =======================================================
    # Optional buzzer command
    # =======================================================

    def play_ready_tune(self):

        try:

            msg = PlayTuneV2()

            # MAVROS PLAY_TUNE_V2 QBASIC1
            msg.format = 1
            msg.tune = "MFT100a8"

            self.tune_pub.publish(msg)

        except Exception:
            pass

    # =======================================================
    # Console
    # =======================================================

    def print_dashboard(self, checks):

        # Refresh one dashboard instead of scrolling ROS logs.
        print("\033[2J\033[H", end="")

        print(
            self.CYAN +
            "============================================================"
            + self.RESET
        )

        print(
            self.WHITE +
            "                 UAV2 CORE SYSTEM"
            + self.RESET
        )

        print(
            self.CYAN +
            "============================================================"
            + self.RESET
        )

        print("")

        for name, ok in checks.items():

            if ok:
                marker = self.GREEN + "[  OK  ]" + self.RESET
            else:
                marker = self.RED + "[ WAIT ]" + self.RESET

            print("{:<16} {}".format(marker, name))

        print("")

        if self.battery is not None:

            voltage = self.battery.voltage
            percentage = self.battery.percentage

            if (
                math.isfinite(voltage)
                and math.isfinite(percentage)
            ):

                print(
                    "Battery         : "
                    "{:.2f} V / {:.0f}%".format(
                        voltage,
                        100.0 * percentage
                    )
                )

        if self.px4_state is not None:

            print(
                "PX4             : mode={} armed={}".format(
                    self.px4_state.mode,
                    self.px4_state.armed
                )
            )

        ch6, ch7 = self.rc_ch6_ch7()

        if ch6 is not None:
            print(
                "RC              : CH6={}  CH7={}".format(
                    ch6,
                    ch7
                )
            )

        print("")

        if self.ready:

            print(
                self.GREEN +
                "############################################################"
                + self.RESET
            )

            print(
                self.GREEN +
                "###################  UAV2 READY FOR ARM  ###################"
                + self.RESET
            )

            print(
                self.GREEN +
                "############################################################"
                + self.RESET
            )

            print("")
            print(
                self.GREEN +
                "Current test: keep CH7 LOW / CH6 HIGH / CH8 SAFE; CH5 ARM only when instructed"
                + self.RESET
            )

        else:

            failed = [
                name
                for name, ok in checks.items()
                if not ok
            ]

            print(
                self.RED +
                "!!!!!!!!!!!!!!!!!!  UAV2 NOT READY  !!!!!!!!!!!!!!!!!!!!!!!!"
                + self.RESET
            )

            print("")
            print(
                self.YELLOW +
                "Waiting for: " +
                ", ".join(failed)
                + self.RESET
            )

        print("")
        print(
            self.WHITE +
            "CH8 KILL switch must remain SAFE. "
            "Ctrl+C stops the operational stack."
            + self.RESET
        )

    # =======================================================
    # Main
    # =======================================================

    def timer_cb(self, event):

        checks = self.evaluate()

        all_ok = all(checks.values())

        now = rospy.Time.now()

        if all_ok:

            if self.ready_since is None:
                self.ready_since = now

            stable = (
                now - self.ready_since
            ).to_sec() >= self.stable_time

        else:

            self.ready_since = None
            stable = False

        self.ready = stable

        self.ready_pub.publish(
            Bool(data=self.ready)
        )

        if self.ready:
            status = "READY_FOR_ARM"
        else:
            failed = [
                key
                for key, value in checks.items()
                if not value
            ]

            status = "NOT_READY: " + ",".join(failed)

        self.status_pub.publish(
            String(data=status)
        )

        # Send optional beep only on NOT READY -> READY edge.
        if self.ready and not self.last_ready:

            self.play_ready_tune()

        self.last_ready = self.ready

        # Dashboard ~2 Hz.
        if (
            now - self.last_console_time
        ).to_sec() >= 0.5:

            self.last_console_time = now
            self.print_dashboard(checks)


if __name__ == "__main__":

    rospy.init_node("system_ready_node")

    SystemReadyNode()

    rospy.spin()
