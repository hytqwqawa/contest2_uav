#!/usr/bin/env python3

import subprocess
import time
from pathlib import Path

import rospy

from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class TakeoffHoverLandDemo:

    def __init__(self):

        self.state = None
        self.odom = None
        self.extended_state = None

        self.health = False
        self.stream = False
        self.mission_allowed = False

        self.point_status = ""
        self.mux_status = ""

        self.range_value = None
        self.range_time = None

        self.ground_z = None

        self.hover_time = float(
            rospy.get_param("~hover_time", 5.0)
        )

        self.takeoff_height = float(
            rospy.get_param(
                "/contest2/point_traj_node/takeoff_height",
                0.80
            )
        )

        self.touchdown_range_threshold = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/range_threshold",
                0.08
            )
        )

        # Final control handoff:
        # stop using our OFFBOARD POINT position controller
        # before physical contact and let PX4 native AUTO.LAND
        # handle the final few centimeters.
        #
        # This is NOT the touchdown threshold.
        self.auto_land_handoff_range = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/autoland_handoff_range",
                0.10
            )
        )

        self.touchdown_range_timeout = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/range_timeout",
                0.30
            )
        )

        self.touchdown_stable_time = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/stable_time",
                0.20
            )
        )

        self.touchdown_min_drop = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/min_drop",
                0.30
            )
        )

        self.touchdown_ground_margin = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/ground_margin",
                0.03
            )
        )

        self.touchdown_vz_max = float(
            rospy.get_param(
                "/contest2/uav2_touchdown/vz_max",
                0.10
            )
        )

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/mavros/extended_state",
            ExtendedState,
            self.extended_state_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=30
        )

        rospy.Subscriber(
            "/contest2/mavros/distance_sensor/hrlv_ez4_pub",
            Range,
            self.range_cb,
            queue_size=30
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/flight_healthy",
            Bool,
            self.health_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/offboard_stream_ready",
            Bool,
            self.stream_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=20
        )


    # ----------------------------------------------------------
    # callbacks
    # ----------------------------------------------------------

    def state_cb(self, msg):
        self.state = msg

    def extended_state_cb(self, msg):
        self.extended_state = msg

    def odom_cb(self, msg):
        self.odom = msg

    def range_cb(self, msg):
        if msg.range > 0.0:
            self.range_value = msg.range
            self.range_time = rospy.Time.now()

    def health_cb(self, msg):
        self.health = msg.data

    def stream_cb(self, msg):
        self.stream = msg.data

    def mission_cb(self, msg):
        self.mission_allowed = msg.data

    def point_cb(self, msg):
        self.point_status = msg.data

    def mux_cb(self, msg):
        self.mux_status = msg.data


    # ----------------------------------------------------------
    # helpers
    # ----------------------------------------------------------

    def z(self):
        if self.odom is None:
            return float("nan")
        return self.odom.pose.pose.position.z

    def vz(self):
        if self.odom is None:
            return float("nan")
        return self.odom.twist.twist.linear.z

    def range_fresh(self):
        return (
            self.range_time is not None
            and self.range_value is not None
            and
            (rospy.Time.now() - self.range_time).to_sec()
            < self.touchdown_range_timeout
        )

    def call(self, short_name, require=True):

        name = (
            "/contest2/ego_uav_interface/"
            + short_name
        )

        try:
            rospy.wait_for_service(name, timeout=2.0)

            r = rospy.ServiceProxy(
                name,
                Trigger
            )()

        except Exception as e:

            if require:
                raise RuntimeError(
                    "%s unavailable: %s"
                    % (name, e)
                )

            return None

        if require and not r.success:
            raise RuntimeError(
                "%s rejected: %s"
                % (name, r.message)
            )

        return r

    def safe_hold(self):

        # Do not change PX4 mode here.
        # Keep POINT selected and freeze the point generator
        # at the measured current position.
        try:
            self.call(
                "abort_trajectory",
                require=False
            )
            self.call(
                "use_point",
                require=False
            )
        except Exception:
            pass

    def fail(self, reason):

        rospy.logerr(
            "DEMO3 ABORT: %s",
            reason
        )

        self.safe_hold()

        if (
            self.state is not None
            and self.state.armed
            and self.state.mode == "OFFBOARD"
        ):
            print()
            print("========================================")
            print(" PILOT TAKEOVER REQUIRED")
            print(" CH7 -> LOW")
            print(" CH6 -> POSCTL")
            print(" Keep CH5 ARM until aircraft is safe.")
            print("========================================")
            print()

            # Do not tear down the POINT backend while the
            # aircraft is still ARMED in OFFBOARD.
            while (
                not rospy.is_shutdown()
                and self.state is not None
                and self.state.armed
                and self.state.mode == "OFFBOARD"
            ):
                rospy.sleep(0.10)

        raise RuntimeError(reason)


    # ----------------------------------------------------------
    # startup
    # ----------------------------------------------------------

    def wait_stack(self):

        deadline = time.time() + 20.0

        while not rospy.is_shutdown():

            ready = (
                self.state is not None
                and self.state.connected
                and self.odom is not None
                and self.health
                and self.stream
                and self.range_fresh()
                and self.point_status != ""
                and self.mux_status != ""
            )

            if ready:
                return

            if time.time() > deadline:
                raise RuntimeError(
                    "stack not ready"
                )

            rospy.sleep(0.10)

    def wait_manual_arm(self):

        print()
        print("========================================")
        print(" DEMO3 READY")
        print(" TAKEOFF -> HOVER -> LAND")
        print("========================================")
        print()
        print("NO SOFTWARE ARM / DISARM")
        print()
        print("WAITING FOR PILOT")
        print(" CH7 : LOW -> MID -> HIGH")
        print(" CH6 : POSCTL")
        print(" CH5 : ARM")
        print()
        print(
            "After ARM, keep CH7 HIGH and CH6 POSCTL "
            "until the script changes mode itself."
        )
        print()

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and self.state.armed
                and self.mission_allowed
                and self.state.mode
                    in ["POSCTL", "OFFBOARD"]
            ):
                print(
                    "[ OK ] PILOT AUTHORIZATION RECEIVED"
                )
                return

            rospy.sleep(0.10)


    # ----------------------------------------------------------
    # takeoff
    # ----------------------------------------------------------

    def run_shared_takeoff(self):

        script = (
            Path.home()
            / "contest2_uav"
            / "runtime"
            / "ego_uav_ws"
            / "src"
            / "ego_uav_interface"
            / "scripts"
            / "uav2_flight_ops_test.py"
        )

        print()
        print("========================================")
        print(" TAKEOFF")
        print("========================================")

        ret = subprocess.call([
            "python3",
            str(script),
            "takeoff"
        ])

        if ret != 0:
            self.fail(
                "shared flight_ops takeoff failed"
            )

    def wait_takeoff_settle(self):

        target_z = (
            self.ground_z
            + self.takeoff_height
        )

        stable_since = None
        deadline = time.time() + 15.0
        last_print = 0.0

        while not rospy.is_shutdown():

            if not self.health:
                self.fail(
                    "flight_healthy lost during takeoff settle"
                )

            if not self.mission_allowed:
                self.fail(
                    "CH7 mission permission lost"
                )

            if (
                self.state is None
                or not self.state.armed
                or self.state.mode != "OFFBOARD"
            ):
                self.fail(
                    "PX4 left normal OFFBOARD state"
                )

            z = self.z()
            vz = self.vz()
            err = z - target_z

            good = (
                self.point_status == "AIRBORNE_HOLD"
                and abs(err) <= 0.06
                and abs(vz) <= 0.10
            )

            now = time.time()

            if now - last_print >= 0.20:
                print(
                    "\rTAKEOFF SETTLE "
                    "z=%+.3f target=%+.3f "
                    "err=%+.3f vz=%+.3f point=%s"
                    %
                    (
                        z,
                        target_z,
                        err,
                        vz,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )
                last_print = now

            if good:

                if stable_since is None:
                    stable_since = now

                elif now - stable_since >= 0.8:
                    print()
                    print(
                        "[ OK ] TAKEOFF PHYSICALLY SETTLED"
                    )
                    return

            else:
                stable_since = None

            if now > deadline:
                print()
                self.fail(
                    "takeoff settle timeout"
                )

            rospy.sleep(0.05)


    # ----------------------------------------------------------
    # hover
    # ----------------------------------------------------------

    def hover(self):

        print()
        print("========================================")
        print(
            " HOVER %.1f s"
            % self.hover_time
        )
        print("========================================")

        deadline = time.time() + self.hover_time
        last_print = 0.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if not self.health:
                self.fail(
                    "flight_healthy lost during hover"
                )

            if not self.mission_allowed:
                self.fail(
                    "CH7 mission permission lost during hover"
                )

            if (
                self.state is None
                or not self.state.armed
                or self.state.mode != "OFFBOARD"
            ):
                self.fail(
                    "PX4 left OFFBOARD during hover"
                )

            if self.mux_status != "POINT_ACTIVE":
                self.fail(
                    "POINT trajectory source not active"
                )

            now = time.time()

            if now - last_print >= 0.25:
                print(
                    "\rHOVER z=%+.3f vz=%+.3f "
                    "remaining=%4.1f s"
                    %
                    (
                        self.z(),
                        self.vz(),
                        max(0.0, deadline - now)
                    ),
                    end="",
                    flush=True
                )
                last_print = now

            rospy.sleep(0.05)

        print()
        print("[ OK ] HOVER COMPLETE")


    # ----------------------------------------------------------
    # landing
    # ----------------------------------------------------------

    def prepare_point_for_land(self):

        self.call(
            "disable_trajectory",
            require=False
        )

        self.call(
            "abort_trajectory",
            require=False
        )

        self.call(
            "use_point",
            require=True
        )

        deadline = time.time() + 2.0

        while not rospy.is_shutdown():

            if self.mux_status == "POINT_ACTIVE":
                return

            if time.time() > deadline:
                raise RuntimeError(
                    "POINT source did not become active"
                )

            rospy.sleep(0.02)

    def request_auto_land_handoff(self):

        print()
        print("========================================")
        print(" FINAL LAND HANDOFF")
        print("========================================")
        print(
            "Strict physical touchdown confirmed."
        )
        print(
            "Leaving OFFBOARD POINT control only now."
        )
        print(
            "Requesting PX4 native AUTO.LAND for "
            "land detection / auto-disarm."
        )
        print()

        response = self.call(
            "request_land",
            require=False
        )

        if response is None:

            print(
                "[WARN] request_land service unavailable."
            )
            return False

        if not response.success:

            print(
                "[WARN] AUTO.LAND request rejected: %s"
                % response.message
            )
            return False

        # Do NOT disable trajectory until PX4 has actually
        # confirmed that it left OFFBOARD.
        deadline = time.time() + 2.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if (
                self.state is not None
                and not self.state.armed
            ):
                self.cleanup_after_disarm()
                return True

            if (
                self.state is not None
                and self.state.mode == "AUTO.LAND"
            ):

                print(
                    "[ OK ] PX4 = AUTO.LAND"
                )

                # PX4 now owns final descent.
                # Our external trajectory is no longer needed.
                self.call(
                    "disable_trajectory",
                    require=False
                )

                self.call(
                    "abort_trajectory",
                    require=False
                )

                return True

            rospy.sleep(0.02)

        print(
            "[WARN] AUTO.LAND request was sent, "
            "but PX4 did not confirm the mode."
        )

        return False


    def wait_px4_auto_disarm(self):

        print()
        print(
            "[ WAIT ] PX4 final landing + auto-disarm..."
        )

        deadline = time.time() + 8.0
        last_print = 0.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            if (
                self.state is not None
                and not self.state.armed
            ):

                self.cleanup_after_disarm()
                return

            # Once AUTO.LAND has been accepted, Demo3 does not
            # attempt to force PX4 back into another mode.
            if (
                self.state is not None
                and self.state.mode != "AUTO.LAND"
            ):

                print()
                print(
                    "[WARN] PX4 left AUTO.LAND -> %s"
                    % self.state.mode
                )

                self.wait_manual_disarm()
                return

            now = time.time()

            if now - last_print >= 0.25:

                px4_ground = (
                    self.extended_state is not None
                    and
                    self.extended_state.landed_state
                    ==
                    ExtendedState.LANDED_STATE_ON_GROUND
                )

                print(
                    "\rPX4 LAND "
                    "range=%s z=%+.3f vz=%+.3f "
                    "armed=%s ground=%s"
                    %
                    (
                        "%.3f" % self.range_value
                        if self.range_value is not None
                        else "NA",
                        self.z(),
                        self.vz(),
                        str(self.state.armed)
                        if self.state is not None
                        else "UNKNOWN",
                        "YES"
                        if px4_ground
                        else "NO"
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            rospy.sleep(0.05)

        print()
        print(
            "[WARN] PX4 AUTO.LAND did not auto-disarm "
            "within timeout."
        )

        self.wait_manual_disarm()


    def wait_manual_disarm(self):

        print()
        print("========================================")
        print(" AUTOMATIC LAND/DISARM NOT COMPLETE")
        print()
        print(" If aircraft is still moving/airborne:")
        print("   use normal pilot takeover:")
        print("   CH7 -> LOW")
        print("   CH6 -> POSCTL")
        print("   keep CH5 ARM until safely landed.")
        print()
        print(" Once physically stable on the ground:")
        print("   CH5 -> DISARM")
        print("========================================")
        print()

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and not self.state.armed
            ):

                self.cleanup_after_disarm()
                return

            rospy.sleep(0.10)


    def cleanup_after_disarm(self):

        print()
        print("========================================")
        print(" VEHICLE DISARMED")
        print(" LANDING COMPLETE")
        print("========================================")

        # Only now, after motors are disarmed, remove the
        # trajectory command. No ARMED mode transition exists.
        self.call(
            "disable_trajectory",
            require=False
        )

        self.call(
            "abort_trajectory",
            require=False
        )

    def land_without_posctl_handoff(self):

        if (
            self.state is None
            or not self.state.armed
        ):

            self.fail(
                "vehicle is not armed before landing"
            )

        if self.state.mode != "OFFBOARD":

            self.fail(
                "landing requires OFFBOARD"
            )

        if not self.health:

            self.fail(
                "flight_healthy false before landing"
            )

        if not self.mission_allowed:

            self.fail(
                "CH7 mission permission false before landing"
            )

        self.prepare_point_for_land()

        if self.state.mode != "OFFBOARD":

            self.fail(
                "lost OFFBOARD during POINT handoff"
            )

        rospy.sleep(0.20)

        z0 = self.z()

        print()
        print("========================================")
        print(" LAND")
        print("========================================")
        print(
            "POINT/OFFBOARD owns descent through touchdown."
        )
        print(
            "Position + yaw control remain active until "
            "strict physical touchdown is confirmed."
        )
        print(
            "Touchdown safety threshold remains %.2f m."
            % self.touchdown_range_threshold
        )
        print(
            "Only AFTER touchdown -> PX4 AUTO.LAND."
        )

        self.call(
            "start_land",
            require=True
        )

        touchdown_since = None
        handoff_attempted = False

        deadline = time.time() + 12.0
        last_print = 0.0

        while not rospy.is_shutdown():

            now = time.time()

            if (
                self.state is not None
                and not self.state.armed
            ):

                self.cleanup_after_disarm()
                return

            # If AUTO.LAND has become active, our own
            # OFFBOARD phase is finished.
            if (
                self.state is not None
                and self.state.mode == "AUTO.LAND"
            ):

                self.call(
                    "disable_trajectory",
                    require=False
                )

                self.call(
                    "abort_trajectory",
                    require=False
                )

                self.wait_px4_auto_disarm()
                return

            if (
                self.state is None
                or self.state.mode != "OFFBOARD"
            ):

                self.fail(
                    "PX4 left OFFBOARD during landing"
                )

            drop = z0 - self.z()
            vz = self.vz()

            # --------------------------------------------------
            # NORMAL FINAL-LANDING PATH
            #
            # At 10 cm range we stop asking our own POINT
            # position controller to deal with the ground.
            #
            # Require a real airborne descent first so an
            # erroneous near-ground range reading cannot cause
            # an immediate LAND handoff.
            # --------------------------------------------------

            # Early range-only AUTO.LAND handoff disabled.
            #
            # Ground testing showed that giving PX4 AUTO.LAND
            # control before contact loses the POINT yaw hold.
            # Keep POINT/OFFBOARD authoritative until the
            # existing strict touchdown gate is satisfied.
            auto_land_gate = False

            if auto_land_gate:

                print()
                print(
                    "[ AUTO.LAND GATE ] "
                    "range=%.3f drop=%.3f "
                    "z=%.3f vz=%+.3f"
                    %
                    (
                        self.range_value,
                        drop,
                        self.z(),
                        vz
                    )
                )

                handoff_attempted = True

                if self.request_auto_land_handoff():

                    if (
                        self.state is not None
                        and not self.state.armed
                    ):
                        return

                    self.wait_px4_auto_disarm()
                    return

                print(
                    "[ FALLBACK ] Continuing existing "
                    "OFFBOARD terminal descent."
                )


            # --------------------------------------------------
            # STRICT TOUCHDOWN FALLBACK
            #
            # This is intentionally still 0.08 m.
            # It is reached only if AUTO.LAND handoff failed.
            # --------------------------------------------------

            px4_on_ground = (
                self.extended_state is not None
                and
                self.extended_state.landed_state
                ==
                ExtendedState.LANDED_STATE_ON_GROUND
            )

            physical_contact = (
                self.ground_z is not None
                and
                self.z()
                    <= (
                        self.ground_z
                        + self.touchdown_ground_margin
                    )
                and
                abs(vz)
                    <= self.touchdown_vz_max
            )

            touchdown_gate = (
                drop >= self.touchdown_min_drop
                and self.range_fresh()
                and self.range_value > 0.0
                and self.range_value
                    <= self.touchdown_range_threshold
                and self.state.armed
                and self.state.mode == "OFFBOARD"
                and (
                    px4_on_ground
                    or physical_contact
                )
            )

            if now - last_print >= 0.20:

                print(
                    "\rLAND "
                    "z=%+.3f drop=%.3f "
                    "vz=%+.3f range=%s "
                    "point=%s"
                    %
                    (
                        self.z(),
                        drop,
                        vz,
                        "%.3f" % self.range_value
                        if self.range_value is not None
                        else "NA",
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now


            if touchdown_gate:

                if touchdown_since is None:

                    touchdown_since = now

                    print()
                    print(
                        "[ TOUCHDOWN FALLBACK CANDIDATE ] "
                        "range=%.3f z=%.3f vz=%+.3f"
                        %
                        (
                            self.range_value,
                            self.z(),
                            vz
                        )
                    )

                elif (
                    now - touchdown_since
                    >= self.touchdown_stable_time
                ):

                    print()
                    print(
                        "[ TOUCHDOWN CONFIRMED ]"
                    )

                    # Keep POINT position/yaw control right up
                    # to confirmed physical contact.
                    #
                    # Once contact is stable, immediately hand
                    # landed detection / auto-disarm to PX4.
                    # No POSCTL transition and no software
                    # DISARM are used.
                    if self.request_auto_land_handoff():

                        if (
                            self.state is not None
                            and not self.state.armed
                        ):
                            return

                        self.wait_px4_auto_disarm()
                        return

                    print(
                        "[WARN] AUTO.LAND handoff failed "
                        "after touchdown."
                    )

                    self.wait_manual_disarm()
                    return

            else:

                touchdown_since = None


            if now > deadline:

                print()

                self.fail(
                    "landing timeout before AUTO.LAND "
                    "or touchdown fallback"
                )

            rospy.sleep(0.02)


    # ----------------------------------------------------------
    # mission
    # ----------------------------------------------------------

    def run(self):

        self.wait_stack()

        self.ground_z = self.z()

        rospy.set_param(
            "/contest2/uav2_touchdown/ground_z_reference",
            self.ground_z
        )

        print(
            "[ GROUND ] z_reference=%.3f"
            % self.ground_z
        )

        self.wait_manual_arm()

        self.run_shared_takeoff()

        self.wait_takeoff_settle()

        self.hover()

        self.land_without_posctl_handoff()

        print()
        print("========================================")
        print(" DEMO3 COMPLETE")
        print("========================================")


if __name__ == "__main__":

    rospy.init_node(
        "takeoff_hover_land_demo"
    )

    node = TakeoffHoverLandDemo()

    try:
        node.run()

    except KeyboardInterrupt:

        rospy.logwarn(
            "DEMO3 interrupted by operator"
        )

        node.safe_hold()

        if (
            node.state is not None
            and node.state.armed
        ):
            print()
            print("========================================")
            print(" DEMO3 INTERRUPTED WHILE ARMED")
            print(" POINT backend will remain running.")
            print(" CH7 -> LOW")
            print(" CH6 -> POSCTL")
            print(" Keep CH5 ARM until aircraft is safe.")
            print(" Then CH5 -> DISARM.")
            print("========================================")
            print()

        raise

    except Exception as e:

        rospy.logerr(
            "DEMO3 FAILED: %s",
            str(e)
        )

        # Best-effort containment for an unexpected exception.
        # Do not intentionally change PX4 mode or disarm here.
        node.safe_hold()

        if (
            node.state is not None
            and node.state.armed
        ):
            print()
            print("========================================")
            print(" DEMO3 ERROR WHILE ARMED")
            print(" POINT backend will remain running.")
            print(" CH7 -> LOW")
            print(" CH6 -> POSCTL")
            print(" Keep CH5 ARM until aircraft is safe.")
            print(" Then CH5 -> DISARM.")
            print("========================================")
            print()

        raise
