#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


class OffboardManager:

    def __init__(self):

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.state_topic = rospy.get_param(
            "~state_topic",
            "/contest2/mavros/state"
        )

        self.health_topic = rospy.get_param(
            "~health_topic",
            "/contest2/ego_uav_interface/flight_healthy"
        )

        self.stream_ready_topic = rospy.get_param(
            "~stream_ready_topic",
            "/contest2/ego_uav_interface/offboard_stream_ready"
        )

        self.rc_offboard_topic = rospy.get_param(
            "~rc_offboard_topic",
            "/contest2/ego_uav_interface/rc_offboard_allowed"
        )

        self.set_mode_service = rospy.get_param(
            "~set_mode_service",
            "/contest2/mavros/set_mode"
        )

        self.state_timeout = rospy.get_param(
            "~state_timeout",
            2.5
        )

        self.health_timeout = rospy.get_param(
            "~health_timeout",
            0.3
        )

        self.stream_timeout = rospy.get_param(
            "~stream_timeout",
            0.3
        )

        self.rc_permission_timeout = rospy.get_param(
            "~rc_permission_timeout",
            0.5
        )

        self.offboard_mode = rospy.get_param(
            "~offboard_mode",
            "OFFBOARD"
        )

        self.fallback_mode = rospy.get_param(
            "~fallback_mode",
            "POSCTL"
        )

        # --------------------------------------------------
        # Internal state
        # --------------------------------------------------

        self.px4_state = None
        self.state_time = None

        self.flight_healthy = False
        self.health_time = None

        self.stream_ready = False
        self.stream_time = None

        # RC autonomy permission.
        #
        # Fail-safe default is FALSE.
        # A fresh TRUE must be received continuously from
        # rc_supervisor.py before OFFBOARD can be requested.
        self.rc_offboard_allowed = False
        self.rc_permission_time = None

        self.requested_mode = None

        # --------------------------------------------------
        # ROS subscribers
        # --------------------------------------------------

        rospy.Subscriber(
            self.state_topic,
            State,
            self.state_callback,
            queue_size=10
        )

        rospy.Subscriber(
            self.health_topic,
            Bool,
            self.health_callback,
            queue_size=10
        )

        rospy.Subscriber(
            self.stream_ready_topic,
            Bool,
            self.stream_callback,
            queue_size=10
        )

        rospy.Subscriber(
            self.rc_offboard_topic,
            Bool,
            self.rc_permission_callback,
            queue_size=10
        )

        # --------------------------------------------------
        # Status publishers
        # --------------------------------------------------

        self.status_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/offboard_manager_status",
            String,
            queue_size=1,
            latch=True
        )

        self.offboard_active_pub = rospy.Publisher(
            "/contest2/ego_uav_interface/offboard_active",
            Bool,
            queue_size=1,
            latch=True
        )

        # --------------------------------------------------
        # User-facing services
        # --------------------------------------------------

        rospy.Service(
            "/contest2/ego_uav_interface/request_offboard",
            Trigger,
            self.request_offboard_callback
        )

        rospy.Service(
            "/contest2/ego_uav_interface/request_posctl",
            Trigger,
            self.request_posctl_callback
        )

        # --------------------------------------------------
        # MAVROS service
        # --------------------------------------------------

        rospy.loginfo(
            "Waiting for MAVROS set_mode service: %s",
            self.set_mode_service
        )

        rospy.wait_for_service(
            self.set_mode_service
        )

        self.set_mode_client = rospy.ServiceProxy(
            self.set_mode_service,
            SetMode
        )

        # --------------------------------------------------
        # Timer
        # --------------------------------------------------

        self.timer = rospy.Timer(
            rospy.Duration(0.1),
            self.timer_callback
        )

        rospy.loginfo("======================================")
        rospy.loginfo(" ego_uav_interface offboard manager")
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" OFFBOARD mode : %s", self.offboard_mode)
        rospy.loginfo(" Fallback mode : %s", self.fallback_mode)
        rospy.loginfo("--------------------------------------")
        rospy.loginfo(" DOES NOT ARM THE VEHICLE")
        rospy.loginfo(" OFFBOARD requires explicit service")
        rospy.loginfo("======================================")

    # ======================================================
    # Callbacks
    # ======================================================

    def state_callback(self, msg):

        self.px4_state = msg
        self.state_time = rospy.Time.now()

    def health_callback(self, msg):

        self.flight_healthy = msg.data
        self.health_time = rospy.Time.now()

    def stream_callback(self, msg):

        self.stream_ready = msg.data
        self.stream_time = rospy.Time.now()

    def rc_permission_callback(self, msg):

        self.rc_offboard_allowed = msg.data
        self.rc_permission_time = rospy.Time.now()

    # ======================================================
    # Freshness checks
    # ======================================================

    def state_fresh(self):

        if self.state_time is None:
            return False

        return (
            rospy.Time.now() - self.state_time
        ).to_sec() < self.state_timeout

    def health_ok(self):

        if self.health_time is None:
            return False

        fresh = (
            rospy.Time.now() - self.health_time
        ).to_sec() < self.health_timeout

        return fresh and self.flight_healthy

    def stream_ok(self):

        if self.stream_time is None:
            return False

        fresh = (
            rospy.Time.now() - self.stream_time
        ).to_sec() < self.stream_timeout

        return fresh and self.stream_ready

    def rc_permission_fresh(self):

        if self.rc_permission_time is None:
            return False

        return (
            rospy.Time.now() - self.rc_permission_time
        ).to_sec() < self.rc_permission_timeout

    def rc_permission_ok(self):

        return (
            self.rc_permission_fresh()
            and self.rc_offboard_allowed
        )

    # ======================================================
    # Mode request
    # ======================================================

    def set_mode(self, mode):

        try:

            result = self.set_mode_client(
                base_mode=0,
                custom_mode=mode
            )

            return result.mode_sent

        except rospy.ServiceException as exc:

            rospy.logerr(
                "set_mode service failed: %s",
                str(exc)
            )

            return False

    # ======================================================
    # User service: request OFFBOARD
    # ======================================================

    def request_offboard_callback(self, req):

        if not self.state_fresh():

            return TriggerResponse(
                success=False,
                message="Rejected: PX4 state timeout"
            )

        if self.px4_state is None or \
           not self.px4_state.connected:

            return TriggerResponse(
                success=False,
                message="Rejected: PX4 not connected"
            )

        if not self.rc_permission_fresh():

            return TriggerResponse(
                success=False,
                message="Rejected: RC OFFBOARD permission unavailable/stale"
            )

        if not self.rc_offboard_allowed:

            return TriggerResponse(
                success=False,
                message="Rejected: RC OFFBOARD permission is false"
            )

        if not self.health_ok():

            return TriggerResponse(
                success=False,
                message="Rejected: flight_healthy is false"
            )

        if not self.stream_ok():

            return TriggerResponse(
                success=False,
                message="Rejected: offboard stream not ready"
            )

        if self.px4_state.mode == self.offboard_mode:

            return TriggerResponse(
                success=True,
                message="Already in OFFBOARD"
            )

        rospy.logwarn(
            "Requesting PX4 mode: %s",
            self.offboard_mode
        )

        success = self.set_mode(
            self.offboard_mode
        )

        if success:

            self.requested_mode = self.offboard_mode

            return TriggerResponse(
                success=True,
                message="OFFBOARD request sent"
            )

        return TriggerResponse(
            success=False,
            message="PX4 rejected OFFBOARD request"
        )

    # ======================================================
    # User service: request POSCTL
    # ======================================================

    def request_posctl_callback(self, req):

        if not self.state_fresh():

            return TriggerResponse(
                success=False,
                message="Rejected: PX4 state timeout"
            )

        if self.px4_state.mode == self.fallback_mode:

            return TriggerResponse(
                success=True,
                message="Already in POSCTL"
            )

        rospy.logwarn(
            "Requesting PX4 mode: %s",
            self.fallback_mode
        )

        success = self.set_mode(
            self.fallback_mode
        )

        if success:

            self.requested_mode = self.fallback_mode

            return TriggerResponse(
                success=True,
                message="POSCTL request sent"
            )

        return TriggerResponse(
            success=False,
            message="PX4 rejected POSCTL request"
        )

    # ======================================================
    # Main monitor
    # ======================================================

    def timer_callback(self, event):

        if not self.state_fresh():

            self.status_pub.publish(
                String(data="WAIT_PX4_STATE")
            )

            self.offboard_active_pub.publish(
                Bool(data=False)
            )

            rospy.logwarn_throttle(
                1.0,
                "[MODE MANAGER] WAIT_PX4_STATE"
            )

            return

        current_mode = self.px4_state.mode

        offboard_active = (
            current_mode == self.offboard_mode
        )

        self.offboard_active_pub.publish(
            Bool(data=offboard_active)
        )

        rc_ok = self.rc_permission_ok()

        # ----------------------------------------------
        # RC autonomy permission is a hard software gate.
        #
        # If OFFBOARD is active and permission disappears
        # or becomes stale, return to POSCTL.
        #
        # This applies whether armed or disarmed.
        # Physical CH6 remains an independent PX4-level
        # manual takeover path.
        # ----------------------------------------------

        if offboard_active and not rc_ok:

            rospy.logerr_throttle(
                1.0,
                "[MODE MANAGER] "
                "OFFBOARD active without fresh RC permission. "
                "Requesting POSCTL."
            )

            self.set_mode(
                self.fallback_mode
            )

            self.status_pub.publish(
                String(
                    data="RC_GATE_FALLBACK_TO_POSCTL"
                )
            )

            return

        # ----------------------------------------------
        # At this stage we are testing DISARMED mode
        # management only.
        #
        # If OFFBOARD is active while DISARMED and our
        # software health disappears, request POSCTL.
        #
        # Armed-flight failsafe policy will be handled
        # separately later together with PX4 failsafes.
        # ----------------------------------------------

        if (
            offboard_active
            and not self.px4_state.armed
            and (
                not self.health_ok()
                or not self.stream_ok()
            )
        ):

            rospy.logerr_throttle(
                1.0,
                "[MODE MANAGER] "
                "OFFBOARD unhealthy while DISARMED. "
                "Requesting POSCTL."
            )

            self.set_mode(
                self.fallback_mode
            )

            self.status_pub.publish(
                String(
                    data="FALLBACK_TO_POSCTL"
                )
            )

            return

        if offboard_active:

            status = "OFFBOARD_ACTIVE"

        elif current_mode == self.fallback_mode:

            if not rc_ok:

                status = "POSCTL_RC_BLOCKED"

            elif self.health_ok() and self.stream_ok():

                status = "POSCTL_READY_FOR_OFFBOARD"

            else:

                status = "POSCTL_NOT_READY"

        else:

            status = (
                "MODE_" + current_mode
            )

        self.status_pub.publish(
            String(data=status)
        )

        rospy.loginfo_throttle(
            1.0,
            "[MODE MANAGER] "
            "mode=%s armed=%s "
            "health=%s stream=%s rc=%s "
            "status=%s",
            current_mode,
            "YES" if self.px4_state.armed else "NO",
            "OK" if self.health_ok() else "BAD",
            "OK" if self.stream_ok() else "BAD",
            "ALLOW" if rc_ok else "BLOCK",
            status
        )


if __name__ == "__main__":

    rospy.init_node(
        "offboard_manager"
    )

    manager = OffboardManager()

    rospy.spin()
