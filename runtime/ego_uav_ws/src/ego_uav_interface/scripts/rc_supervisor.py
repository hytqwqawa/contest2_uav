#!/usr/bin/env python3

import rospy

from mavros_msgs.msg import RCIn, State
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class RCSupervisor:

    LOW_MAX = 1300
    HIGH_MIN = 1700

    RC_TIMEOUT = 1.5
    SWITCH_STABLE_TIME = 0.15

    def __init__(self):

        # ====================================================
        # ROS state
        # ====================================================

        self.rc_msg = None
        self.rc_time = None

        self.mavros_state = None
        self.state_time = None

        # Raw / stable switch states
        self.ch6_candidate = "UNKNOWN"
        self.ch7_candidate = "UNKNOWN"

        self.ch6_candidate_since = rospy.Time.now()
        self.ch7_candidate_since = rospy.Time.now()

        self.ch6_state = "UNKNOWN"
        self.ch7_state = "UNKNOWN"

        self.last_ch6_state = "UNKNOWN"
        self.last_ch7_state = "UNKNOWN"

        self.ch5_pwm = 0
        self.ch6_pwm = 0
        self.ch7_pwm = 0
        self.ch8_pwm = 0

        # ====================================================
        # Safety latch
        #
        # Startup is ALWAYS latched.
        #
        # The operator must deliberately perform:
        #
        #   CH7 LOW -> MID
        #
        # before software autonomy is permitted.
        # ====================================================

        self.manual_latched = True
        self.seen_ch7_low = False

        self.offboard_allowed = False
        self.mission_allowed = False

        self.status = "WAIT_RC"

        # Avoid repeated service flooding
        self.last_posctl_request = rospy.Time(0)

        # ====================================================
        # Subscribers
        # ====================================================

        rospy.Subscriber(
            "/contest2/mavros/rc/in",
            RCIn,
            self.rc_callback,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_callback,
            queue_size=20
        )

        # ====================================================
        # Publishers
        # ====================================================

        self.offboard_allowed_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/rc_offboard_allowed",
            Bool,
            queue_size=1,
            latch=True
        )

        self.mission_allowed_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/rc_mission_allowed",
            Bool,
            queue_size=1,
            latch=True
        )

        self.manual_latched_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/rc_manual_latched",
            Bool,
            queue_size=1,
            latch=True
        )

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/rc_supervisor_status",
            String,
            queue_size=1,
            latch=True
        )

        # ====================================================
        # Optional service proxies
        #
        # We intentionally do NOT wait for these services.
        # If one is absent, the supervisor still runs.
        # ====================================================

        self.request_posctl_srv = rospy.ServiceProxy(
            "/contest2/ego_uav_interface/request_posctl",
            Trigger
        )

        self.disable_trajectory_srv = rospy.ServiceProxy(
            "/contest2/ego_uav_interface/disable_trajectory",
            Trigger
        )

        self.abort_trajectory_srv = rospy.ServiceProxy(
            "/contest2/ego_uav_interface/abort_trajectory",
            Trigger
        )

        self.timer = rospy.Timer(
            rospy.Duration(0.05),   # 20 Hz
            self.timer_callback
        )

        rospy.logwarn(
            "RC supervisor started in MANUAL-LATCHED state."
        )

        rospy.logwarn(
            "Autonomy requires deliberate CH7 LOW -> MID."
        )

        rospy.logwarn(
            "This node DOES NOT arm, disarm, kill, takeoff, "
            "or automatically enter OFFBOARD."
        )

    # ========================================================
    # Callbacks
    # ========================================================

    def rc_callback(self, msg):

        self.rc_msg = msg
        self.rc_time = rospy.Time.now()

        # Need at least CH1...CH8
        if len(msg.channels) < 8:
            return

        self.ch5_pwm = msg.channels[4]
        self.ch6_pwm = msg.channels[5]
        self.ch7_pwm = msg.channels[6]
        self.ch8_pwm = msg.channels[7]

        now = rospy.Time.now()

        ch6_new = self.classify_switch(self.ch6_pwm)
        ch7_new = self.classify_switch(self.ch7_pwm)

        if ch6_new != self.ch6_candidate:
            self.ch6_candidate = ch6_new
            self.ch6_candidate_since = now

        if ch7_new != self.ch7_candidate:
            self.ch7_candidate = ch7_new
            self.ch7_candidate_since = now

    def state_callback(self, msg):

        self.mavros_state = msg
        self.state_time = rospy.Time.now()

    # ========================================================
    # Helpers
    # ========================================================

    @staticmethod
    def classify_switch(pwm):

        if pwm <= RCSupervisor.LOW_MAX:
            return "LOW"

        if pwm >= RCSupervisor.HIGH_MIN:
            return "HIGH"

        return "MID"

    def rc_fresh(self):

        if self.rc_time is None:
            return False

        return (
            rospy.Time.now() - self.rc_time
        ).to_sec() < self.RC_TIMEOUT

    def update_stable_switches(self):

        now = rospy.Time.now()

        if (
            self.ch6_candidate != self.ch6_state and
            (now - self.ch6_candidate_since).to_sec()
            >= self.SWITCH_STABLE_TIME
        ):
            self.last_ch6_state = self.ch6_state
            self.ch6_state = self.ch6_candidate

            rospy.logwarn(
                "CH6 stable transition: %s -> %s",
                self.last_ch6_state,
                self.ch6_state
            )

            self.handle_ch6_transition()

        if (
            self.ch7_candidate != self.ch7_state and
            (now - self.ch7_candidate_since).to_sec()
            >= self.SWITCH_STABLE_TIME
        ):
            self.last_ch7_state = self.ch7_state
            self.ch7_state = self.ch7_candidate

            rospy.logwarn(
                "CH7 stable transition: %s -> %s",
                self.last_ch7_state,
                self.ch7_state
            )

            self.handle_ch7_transition()

    # ========================================================
    # Safety actions
    # ========================================================

    def try_disable_trajectory(self):

        try:
            self.disable_trajectory_srv()
        except rospy.ServiceException:
            pass

    def try_abort_trajectory(self):

        try:
            self.abort_trajectory_srv()
        except rospy.ServiceException:
            pass

    def request_posctl_if_needed(self):

        if self.mavros_state is None:
            return

        # Only force POSCTL when the physical CH6 switch itself
        # is in the POSCTL/HIGH position.
        #
        # If the pilot selected ALTCTL/STABILIZED with CH6,
        # never override that selection from ROS.

        if self.ch6_state != "HIGH":
            return

        if self.mavros_state.mode != "OFFBOARD":
            return

        now = rospy.Time.now()

        if (
            now - self.last_posctl_request
        ).to_sec() < 0.5:
            return

        self.last_posctl_request = now

        try:
            response = self.request_posctl_srv()

            if response.success:
                rospy.logwarn(
                    "RC supervisor requested POSCTL."
                )
            else:
                rospy.logerr(
                    "POSCTL request rejected: %s",
                    response.message
                )

        except rospy.ServiceException as e:

            rospy.logerr_throttle(
                1.0,
                "Cannot request POSCTL: %s",
                str(e)
            )

    def latch_manual(self, reason):

        if not self.manual_latched:
            rospy.logerr(
                "AUTONOMY REVOKED: %s",
                reason
            )

        self.manual_latched = True

        self.offboard_allowed = False
        self.mission_allowed = False

        self.try_abort_trajectory()
        self.try_disable_trajectory()

    # ========================================================
    # Switch transitions
    # ========================================================

    def handle_ch6_transition(self):

        # CH6 HIGH is the verified physical POSCTL switch position.
        #
        # Moving CH6 away from HIGH is interpreted as direct
        # pilot takeover at the PX4 level.

        if self.ch6_state != "HIGH":

            self.latch_manual(
                "CH6 left POSCTL selector position"
            )

            rospy.logerr(
                "CH6 manual takeover detected. "
                "Autonomy remains latched even if CH6 returns HIGH."
            )

    def handle_ch7_transition(self):

        # ----------------------------------------------------
        # LOW = explicit software SAFE / manual state
        # ----------------------------------------------------

        if self.ch7_state == "LOW":

            self.seen_ch7_low = True

            self.latch_manual(
                "CH7 LOW"
            )

            rospy.logwarn(
                "CH7 LOW: MANUAL_SAFE"
            )

            return

        # ----------------------------------------------------
        # MID = deliberate re-authorization point
        #
        # Must have seen LOW first, and CH6 must physically
        # remain in its POSCTL/HIGH position.
        # ----------------------------------------------------

        if self.ch7_state == "MID":

            if (
                self.seen_ch7_low and
                self.ch6_state == "HIGH"
            ):

                self.manual_latched = False

                rospy.logwarn(
                    "CH7 LOW -> MID authorization accepted."
                )

            else:

                self.manual_latched = True

                rospy.logwarn(
                    "CH7 MID blocked: need CH7 LOW first "
                    "and CH6 HIGH."
                )

            # Dropping HIGH -> MID means:
            # cancel active mission trajectory but retain the
            # possibility of OFFBOARD HOLD.
            if self.last_ch7_state == "HIGH":
                self.try_abort_trajectory()

            return

        # ----------------------------------------------------
        # HIGH = mission permission only
        #
        # HIGH itself never starts a mission.
        # ----------------------------------------------------

        if self.ch7_state == "HIGH":

            if self.manual_latched:

                rospy.logwarn(
                    "CH7 HIGH blocked: autonomy is latched. "
                    "Use CH7 LOW -> MID first."
                )

            else:

                rospy.logwarn(
                    "CH7 HIGH: mission permission available."
                )

    # ========================================================
    # Main logic
    # ========================================================

    def timer_callback(self, event):

        if not self.rc_fresh():

            self.manual_latched = True
            self.offboard_allowed = False
            self.mission_allowed = False

            self.status = "RC_LOST_MANUAL_LATCHED"

            self.try_disable_trajectory()

            self.publish_status()
            return

        self.update_stable_switches()

        # ----------------------------------------------------
        # Permission computation
        # ----------------------------------------------------

        ch6_ok = (
            self.ch6_state == "HIGH"
        )

        authorized = (
            not self.manual_latched and
            ch6_ok
        )

        self.offboard_allowed = (
            authorized and
            self.ch7_state in ("MID", "HIGH")
        )

        self.mission_allowed = (
            authorized and
            self.ch7_state == "HIGH"
        )

        # ----------------------------------------------------
        # Explicit CH7 LOW:
        # if PX4 is still OFFBOARD and CH6 itself remains in
        # POSCTL position, request POSCTL.
        # ----------------------------------------------------

        if self.ch7_state == "LOW":

            self.request_posctl_if_needed()

        # ----------------------------------------------------
        # Status
        # ----------------------------------------------------

        if self.ch6_state != "HIGH":

            self.status = "MANUAL_OVERRIDE_LATCHED"

        elif self.manual_latched:

            if self.ch7_state == "HIGH":
                self.status = "HIGH_BLOCKED_NEED_LOW_MID"
            else:
                self.status = "MANUAL_SAFE_LATCHED"

        elif self.ch7_state == "MID":

            self.status = "AUTO_HOLD_PERMISSION"

        elif self.ch7_state == "HIGH":

            self.status = "AUTO_MISSION_PERMISSION"

        else:

            self.status = "WAIT_VALID_SWITCH"

        self.publish_status()

    def publish_status(self):

        self.offboard_allowed_pub.publish(
            Bool(data=self.offboard_allowed)
        )

        self.mission_allowed_pub.publish(
            Bool(data=self.mission_allowed)
        )

        self.manual_latched_pub.publish(
            Bool(data=self.manual_latched)
        )

        mode = "UNKNOWN"

        if self.mavros_state is not None:
            mode = self.mavros_state.mode

        text = (
            "{} | "
            "CH5={} "
            "CH6={}({}) "
            "CH7={}({}) "
            "CH8={} "
            "PX4={} "
            "OFFB_ALLOW={} "
            "MISSION_ALLOW={}"
        ).format(
            self.status,
            self.ch5_pwm,
            self.ch6_state,
            self.ch6_pwm,
            self.ch7_state,
            self.ch7_pwm,
            self.ch8_pwm,
            mode,
            self.offboard_allowed,
            self.mission_allowed
        )

        self.status_pub.publish(
            String(data=text)
        )


if __name__ == "__main__":

    rospy.init_node("rc_supervisor")

    RCSupervisor()

    rospy.spin()
