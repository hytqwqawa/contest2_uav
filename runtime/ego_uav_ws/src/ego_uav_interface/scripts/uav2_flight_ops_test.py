#!/usr/bin/env python3

import sys
import time
import rospy

from mavros_msgs.msg import State, ExtendedState
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


GREEN = "\033[1;32m"
RED = "\033[1;31m"
CYAN = "\033[1;36m"
YELLOW = "\033[1;33m"
RESET = "\033[0m"


class FlightOpsTest:

    def __init__(self):

        self.state = None
        self.odom = None

        self.health = None
        self.stream = None
        self.mission = None

        self.point_status = None
        self.mux_status = None

        # POST_ARM_REQUALIFY_CLIENT_V1
        #
        # offboard_manager is the authoritative OFFBOARD gate.
        # This client only waits until that gate reports that the
        # current ARM session has been re-qualified.
        self.offboard_manager_status = None
        self.offboard_manager_status_time = None

        # POST_ARM_REQUALIFY_CLIENT_V2

        # UAV2 touchdown state.
        self.extended_state = None
        self.range_value = None
        self.range_time = None

        # TFMini normally reads about 0.04-0.06 m on the floor.
        self.touchdown_range_threshold = rospy.get_param(
            "/contest2/uav2_touchdown/range_threshold",
            0.08
        )

        self.touchdown_range_timeout = rospy.get_param(
            "/contest2/uav2_touchdown/range_timeout",
            0.30
        )

        self.touchdown_stable_time = rospy.get_param(
            "/contest2/uav2_touchdown/stable_time",
            0.10
        )

        # Never allow the touchdown gate immediately after takeoff.
        self.touchdown_min_drop = rospy.get_param(
            "/contest2/uav2_touchdown/min_drop",
            0.30
        )

        # Actual PX4 local-Z before takeoff.
        self.ground_z_reference = rospy.get_param(
            "/contest2/uav2_touchdown/ground_z_reference",
            1.0e9
        )

        # Physical-contact fallback.
        # TFMini alone is never sufficient to disarm.
        self.touchdown_ground_margin = rospy.get_param(
            "/contest2/uav2_touchdown/ground_margin",
            0.03
        )

        self.touchdown_vz_max = rospy.get_param(
            "/contest2/uav2_touchdown/vz_max",
            0.10
        )

        # Hand the final few centimeters to PX4 Land mode.
        #
        # TFMini reads about 0.04-0.06 m when physically landed,
        # so 0.10 m leaves only a few centimeters for AUTO.LAND.
        self.autoland_handoff_range = rospy.get_param(
            "/contest2/uav2_touchdown/autoland_handoff_range",
            0.10
        )

        self.autoland_timeout = rospy.get_param(
            "/contest2/uav2_touchdown/autoland_timeout",
            6.0
        )

        rospy.Subscriber(
            "/contest2/mavros/state",
            State,
            self.state_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/mavros/extended_state",
            ExtendedState,
            self.extended_state_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/mavros/distance_sensor/hrlv_ez4_pub",
            Range,
            self.range_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest2/mavros/local_position/odom",
            Odometry,
            self.odom_cb,
            queue_size=20
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
            "/contest2/ego_uav_interface/rc_mission_allowed",
            Bool,
            self.mission_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/point_traj_status",
            String,
            self.point_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/trajectory_mux_status",
            String,
            self.mux_cb,
            queue_size=10
        )

        rospy.Subscriber(
            "/contest2/ego_uav_interface/offboard_manager_status",
            String,
            self.offboard_manager_status_cb,
            queue_size=10
        )

        self.services = {}

        names = [
            "request_offboard",
            "request_posctl",
            "disable_trajectory",
            "use_point",
            "abort_trajectory",
            "start_takeoff",
            "start_land",
        ]

        for name in names:

            full = "/contest2/ego_uav_interface/" + name

            rospy.wait_for_service(
                full,
                timeout=5.0
            )

            self.services[name] = rospy.ServiceProxy(
                full,
                Trigger
            )


    def state_cb(self, msg):
        self.state = msg


    def extended_state_cb(self, msg):
        self.extended_state = msg


    def range_cb(self, msg):
        self.range_value = msg.range
        self.range_time = rospy.Time.now()


    def odom_cb(self, msg):
        self.odom = msg


    def health_cb(self, msg):
        self.health = msg.data


    def stream_cb(self, msg):
        self.stream = msg.data


    def mission_cb(self, msg):
        self.mission = msg.data


    def point_cb(self, msg):
        self.point_status = msg.data


    def mux_cb(self, msg):
        self.mux_status = msg.data


    def offboard_manager_status_cb(self, msg):
        self.offboard_manager_status = msg.data
        self.offboard_manager_status_time = rospy.Time.now()


    def z(self):

        if self.odom is None:
            return float("nan")

        return self.odom.pose.pose.position.z


    def range_fresh(self):

        return (
            self.range_time is not None
            and
            (rospy.Time.now() - self.range_time).to_sec()
            < self.touchdown_range_timeout
        )


    def wait_topics(self):

        deadline = time.time() + 4.0

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and self.odom is not None
                and self.health is not None
                and self.stream is not None
                and self.mission is not None
                and self.point_status is not None
                and self.mux_status is not None
                and self.offboard_manager_status is not None
                and self.extended_state is not None
                and self.range_value is not None
            ):
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.02)

        return False


    def wait_mode(self, mode, timeout):

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            if (
                self.state is not None
                and self.state.mode == mode
            ):
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.02)

        return False


    def wait_post_arm_requalified(self, timeout=10.0):

        print()
        print(
            CYAN +
            "[ WAIT ] POST-ARM estimator requalification..."
            + RESET
        )

        deadline = time.time() + timeout
        last_status = None

        # POST_ARM_REQUALIFY_CLIENT_V3
        #
        # A newly launched client may initially receive a latched
        # pre-ARM status. Accept only a manager status received after
        # this wait began, and only the explicit ARMED-ready state.
        #
        # This also permits a client that starts after the 5-s
        # settling interval has already completed.
        wait_start = rospy.Time.now()

        while not rospy.is_shutdown():

            if self.state is None or not self.state.armed:
                print(
                    RED +
                    "[ABORT] vehicle disarmed during "
                    "POST-ARM requalification"
                    + RESET
                )
                return False

            if self.state.mode != "POSCTL":
                print(
                    RED +
                    "[ABORT] expected POSCTL during "
                    "POST-ARM requalification, mode=%s"
                    % self.state.mode
                    + RESET
                )
                return False

            status = self.offboard_manager_status

            status_received_after_wait_start = (
                self.offboard_manager_status_time is not None
                and
                self.offboard_manager_status_time >= wait_start
            )

            if status != last_status:
                print(
                    "        manager=%s  health=%s  stream=%s"
                    %
                    (
                        status,
                        self.health,
                        self.stream
                    )
                )
                last_status = status

            if (
                status == "POSCTL_ARMED_READY_FOR_OFFBOARD"
                and status_received_after_wait_start
            ):
                print(
                    GREEN +
                    "[ OK ] POST-ARM requalification complete"
                    + RESET
                )
                return True

            if time.time() > deadline:
                print(
                    RED +
                    "[ABORT] POST-ARM requalification timeout; "
                    "manager=%s health=%s stream=%s"
                    %
                    (
                        status,
                        self.health,
                        self.stream
                    )
                    + RESET
                )
                return False

            rospy.sleep(0.05)

        return False


    def wait_mux_point(self, timeout=1.0):

        deadline = time.time() + timeout

        while not rospy.is_shutdown():

            if self.mux_status == "POINT_ACTIVE":
                return True

            if time.time() > deadline:
                return False

            rospy.sleep(0.02)

        return False


    def call(self, name, require=True):

        try:

            r = self.services[name]()

        except rospy.ServiceException as exc:

            print(
                RED +
                "[SERVICE ERROR] %s: %s"
                % (name, str(exc))
                + RESET
            )

            return None

        print(
            "%-20s success=%s  %s"
            %
            (
                name,
                r.success,
                r.message
            )
        )

        if require and not r.success:
            return None

        return r


    def show_initial(self):

        print()
        print("========================================")
        print(" FLIGHT OPS TEST")
        print("========================================")

        print("armed          :", self.state.armed)
        print("mode           :", self.state.mode)
        print("health         :", self.health)
        print("stream         :", self.stream)
        print("mission        :", self.mission)
        print("point          :", self.point_status)
        print("mux            :", self.mux_status)
        print("z              : %.3f" % self.z())
        print()


    def prepare_point_control(self):

        print(
            CYAN +
            "[1] Preparing POINT control source..."
            + RESET
        )

        # Stop whatever trajectory is currently reaching PX4.
        r = self.call(
            "disable_trajectory",
            require=False
        )

        rospy.sleep(0.10)

        # Capture the CURRENT aircraft position into point node.
        # This is critical after an EGO mission.
        r = self.call(
            "abort_trajectory",
            require=True
        )

        if r is None:
            return False

        rospy.sleep(0.10)

        r = self.call(
            "use_point",
            require=True
        )

        if r is None:
            return False

        if not self.wait_mux_point(1.0):

            print(
                RED +
                "[ABORT] mux did not become POINT_ACTIVE"
                + RESET
            )

            return False

        print(
            GREEN +
            "[ OK ] mux = POINT_ACTIVE"
            + RESET
        )

        return True


    def ensure_offboard(self):

        if self.state.mode == "OFFBOARD":
            return True

        if self.state.mode != "POSCTL":

            print(
                RED +
                "[ABORT] PX4 must be POSCTL or OFFBOARD. mode=%s"
                % self.state.mode
                + RESET
            )

            return False

        print(
            CYAN +
            "[2] Requesting OFFBOARD..."
            + RESET
        )

        r = self.call(
            "request_offboard",
            require=True
        )

        if r is None:
            return False

        if not self.wait_mode(
            "OFFBOARD",
            1.5
        ):

            print(
                RED +
                "[ABORT] PX4 did not enter OFFBOARD"
                + RESET
            )

            return False

        # Give point_traj/offboard_manager callbacks
        # a short time to observe the confirmed mode.
        rospy.sleep(0.25)

        print(
            GREEN +
            "[ OK ] PX4 = OFFBOARD"
            + RESET
        )

        return True


    def takeoff(self):

        self.show_initial()

        if not self.state.armed:

            print(
                RED +
                "[ABORT] ARM with physical CH5 first."
                + RESET
            )

            return 1

        # Do not make an instantaneous health decision here.
        # ARM can temporarily make flight_healthy False.
        # wait_post_arm_requalified() requires the central
        # offboard_manager to observe the ARM session and restore
        # full OFFBOARD eligibility before this client proceeds.

        if not self.stream:

            print(
                RED +
                "[ABORT] OFFBOARD stream not ready"
                + RESET
            )

            return 1

        if not self.mission:

            print(
                RED +
                "[ABORT] CH7 mission permission is FALSE"
                + RESET
            )

            return 1

        if self.point_status not in [
            "HOLD_GROUND",
            "LANDED_HOLD"
        ]:

            print(
                RED +
                "[ABORT] point state is not ground HOLD: %s"
                % self.point_status
                + RESET
            )

            return 1

        if not self.wait_post_arm_requalified():
            return 1

        if not self.prepare_point_control():
            return 1

        if not self.ensure_offboard():
            return 1

        print(
            CYAN +
            "[3] Starting TAKEOFF..."
            + RESET
        )

        # Retry ONLY the known OFFBOARD callback synchronization race.
        accepted = False

        for attempt in range(1, 4):

            r = self.call(
                "start_takeoff",
                require=False
            )

            if r is not None and r.success:

                accepted = True
                break

            msg = (
                r.message
                if r is not None
                else ""
            )

            if (
                "ARMED and OFFBOARD" not in msg
                and
                "connected, ARMED and OFFBOARD" not in msg
            ):

                print(
                    RED +
                    "[ABORT] takeoff rejected for a real reason"
                    + RESET
                )

                return 1

            print(
                YELLOW +
                "[ retry %d/3 ] waiting for OFFBOARD callback sync..."
                % attempt
                + RESET
            )

            rospy.sleep(0.25)

        if not accepted:

            print(
                RED +
                "[ABORT] TAKEOFF not accepted after retries"
                + RESET
            )

            return 1

        z0 = self.z()

        print(
            GREEN +
            "[ ACCEPTED ] TAKEOFF z0=%.3f"
            % z0
            + RESET
        )

        deadline = time.time() + 9.0
        last_print = 0.0

        while not rospy.is_shutdown():

            now = time.time()

            if (
                now - last_print
                >= 0.10
            ):

                print(
                    "\r\033[2K"
                    "TAKEOFF  z=%6.3f  dz=%5.3f  "
                    "mode=%-8s  point=%s"
                    %
                    (
                        self.z(),
                        self.z() - z0,
                        self.state.mode,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            if not self.state.armed:

                print()
                print(
                    RED +
                    "[ABORT] vehicle disarmed during takeoff"
                    + RESET
                )
                return 1

            if self.state.mode != "OFFBOARD":

                print()
                print(
                    RED +
                    "[ABORT] PX4 left OFFBOARD -> %s"
                    % self.state.mode
                    + RESET
                )
                return 1

            if self.point_status == "AIRBORNE_HOLD":

                print()

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                print(
                    GREEN +
                    " TAKEOFF COMPLETE  z=%.3f"
                    % self.z()
                    + RESET
                )

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                return 0

            if now > deadline:

                print()

                print(
                    RED +
                    "[ABORT] takeoff timeout, point=%s z=%.3f"
                    %
                    (
                        self.point_status,
                        self.z()
                    )
                    + RESET
                )

                return 1

            rospy.sleep(0.02)

        return 1


    def land(self):

        self.show_initial()

        if not self.state.armed:

            print(
                RED +
                "[ABORT] vehicle is not armed"
                + RESET
            )
            return 1

        if not self.health or not self.mission:

            print(
                RED +
                "[ABORT] health/mission permission invalid"
                + RESET
            )
            return 1

        if self.state.mode != "OFFBOARD":

            print(
                RED +
                "[ABORT] landing test starts from OFFBOARD. mode=%s"
                % self.state.mode
                + RESET
            )
            return 1

        z0 = self.z()
        touchdown_since = None
        last_disarm_try = 0.0

        if not self.prepare_point_control():
            return 1

        # Still require OFFBOARD after source handoff.
        if self.state.mode != "OFFBOARD":

            print(
                RED +
                "[ABORT] lost OFFBOARD during point handoff"
                + RESET
            )
            return 1

        rospy.sleep(0.20)

        print(
            CYAN +
            "[2] Starting LAND..."
            + RESET
        )

        r = self.call(
            "start_land",
            require=True
        )

        if r is None:
            return 1

        print(
            GREEN +
            "[ ACCEPTED ] LAND z0=%.3f"
            % z0
            + RESET
        )

        deadline = time.time() + 9.0
        last_print = 0.0

        while not rospy.is_shutdown():

            now = time.time()

            if (
                now - last_print
                >= 0.10
            ):

                print(
                    "\r\033[2K"
                    "LAND     z=%6.3f  drop=%5.3f  "
                    "mode=%-8s  point=%s"
                    %
                    (
                        self.z(),
                        z0 - self.z(),
                        self.state.mode,
                        self.point_status
                    ),
                    end="",
                    flush=True
                )

                last_print = now

            # --------------------------------------------------
            # UAV2 FINAL TOUCHDOWN
            #
            # Keep OFFBOARD active all the way to physical
            # touchdown so POINT continues to hold XY and yaw.
            #
            # Disarm only after:
            #   1) sufficient descent from the airborne state;
            #   2) fresh TFMini range <= threshold;
            #   3) PX4 land detector reports ON_GROUND;
            #   4) the condition remains true for a short time.
            #
            # No AUTO.LAND handoff is used.
            # --------------------------------------------------

            now = time.time()

            drop = z0 - self.z()

            vz = (
                self.odom.twist.twist.linear.z
                if self.odom is not None
                else 999.0
            )

            px4_on_ground = (
                self.extended_state is not None
                and self.extended_state.landed_state
                    == ExtendedState.LANDED_STATE_ON_GROUND
            )

            ground_ref_valid = (
                self.ground_z_reference < 1.0e8
            )

            physical_contact = (
                ground_ref_valid
                and self.z()
                    <= (
                        self.ground_z_reference
                        + self.touchdown_ground_margin
                    )
                and abs(vz)
                    <= self.touchdown_vz_max
            )

            touchdown_gate = (
                drop >= self.touchdown_min_drop
                and self.range_fresh()
                and self.range_value > 0.0
                and self.range_value
                    <= self.touchdown_range_threshold
                and self.state is not None
                and self.state.armed
                and self.state.mode == "OFFBOARD"
                and (
                    px4_on_ground
                    or physical_contact
                )
            )

            if touchdown_gate:

                if touchdown_since is None:

                    touchdown_since = now

                    print()
                    print(
                        CYAN +
                        "[ TOUCHDOWN CANDIDATE ] "
                        "range=%.3f z=%.3f "
                        "ground=%.3f vz=%+.3f "
                        "px4_ground=%s"
                        %
                        (
                            self.range_value,
                            self.z(),
                            self.ground_z_reference,
                            vz,
                            "YES"
                            if px4_on_ground
                            else "NO"
                        )
                        + RESET
                    )

                elif (
                    now - touchdown_since
                    >= self.touchdown_stable_time
                ):

                    print()

                    print(
                        CYAN +
                        "[ TOUCHDOWN CONFIRMED ] "
                        "range=%.3f z=%.3f "
                        "px4_ground=%s"
                        %
                        (
                            self.range_value,
                            self.z(),
                            "YES"
                            if px4_on_ground
                            else "NO"
                        )
                        + RESET
                    )

                    # ------------------------------------------
                    # ------------------------------------------
                    # TOUCHDOWN_OFFBOARD_SETTLE_V4
                    #
                    # Physical contact is confirmed, but do NOT
                    # change flight mode yet.
                    #
                    # Keep the existing LAND_TERMINAL_DESCENT
                    # command briefly. XY/yaw therefore remain
                    # under the same OFFBOARD controller, while
                    # the downward reference continues to reduce
                    # thrust and suppress landing-gear rebound.
                    #
                    # Exit early if PX4 itself reports ON_GROUND.
                    # ------------------------------------------

                    print(
                        "[3] Touchdown confirmed; "
                        "keeping OFFBOARD terminal settle..."
                    )

                    settle_deadline = time.time() + 0.35
                    px4_ground_seen = px4_on_ground

                    while (
                        not rospy.is_shutdown()
                        and time.time() < settle_deadline
                    ):

                        if (
                            self.state is not None
                            and not self.state.armed
                        ):

                            print()
                            print(
                                GREEN +
                                "========================================\n"
                                " LANDING COMPLETE - PX4 DISARMED\n"
                                "========================================"
                                + RESET
                            )

                            return 0

                        px4_ground_now = (
                            self.extended_state is not None
                            and self.extended_state.landed_state
                                == ExtendedState.LANDED_STATE_ON_GROUND
                        )

                        if px4_ground_now:

                            px4_ground_seen = True
                            break

                        rospy.sleep(0.02)


                    # ------------------------------------------
                    # Now stop the downward trajectory.
                    # No AUTO.LAND mode is ever requested.
                    # ------------------------------------------

                    print(
                        "[4] Freezing ground contact..."
                    )

                    self.call(
                        "disable_trajectory",
                        require=False
                    )

                    self.call(
                        "abort_trajectory",
                        require=False
                    )

                    rospy.sleep(0.05)

                    if px4_ground_seen:

                        print(
                            "[ OK ] PX4 ground detector = ON_GROUND"
                        )

                    else:

                        print(
                            "[ INFO ] PX4 ground detector not yet "
                            "ON_GROUND; using POSCTL finish"
                        )


                    # ------------------------------------------
                    # Preserve the previously proven finishing
                    # path. Mode change occurs only after the
                    # OFFBOARD touchdown-settle interval.
                    # ------------------------------------------

                    print(
                        "[4b] Requesting POSCTL after "
                        "touchdown settle..."
                    )

                    self.call(
                        "request_posctl",
                        require=False
                    )

                    mode_deadline = time.time() + 1.5

                    while (
                        not rospy.is_shutdown()
                        and time.time() < mode_deadline
                    ):

                        if (
                            self.state is not None
                            and not self.state.armed
                        ):
                            break

                        if (
                            self.state is not None
                            and self.state.mode == "POSCTL"
                        ):
                            break

                        rospy.sleep(0.02)


                    print(
                        "[5] Waiting for PX4 land detector "
                        "and auto-disarm..."
                    )


                    # COM_DISARM_LAND is currently 2 s.
                    # Allow margin for land detection itself.
                    disarm_deadline = time.time() + 6.0
                    last_print = 0.0

                    while (
                        not rospy.is_shutdown()
                        and time.time() < disarm_deadline
                    ):

                        if (
                            self.state is not None
                            and not self.state.armed
                        ):
                            print()
                            print(
                                GREEN +
                                "========================================\n"
                                " LANDING COMPLETE - PX4 DISARMED\n"
                                "========================================"
                                + RESET
                            )

                            return 0


                        now_print = time.time()

                        if now_print - last_print >= 0.5:

                            px4_ground_now = (
                                self.extended_state is not None
                                and self.extended_state.landed_state
                                    == ExtendedState.LANDED_STATE_ON_GROUND
                            )

                            print(
                                "GROUND   z=% .3f  range=%.3f  "
                                "armed=%s mode=%s px4_ground=%s"
                                %
                                (
                                    self.z(),
                                    self.range_value,
                                    str(self.state.armed)
                                    if self.state is not None
                                    else "UNKNOWN",
                                    self.state.mode
                                    if self.state is not None
                                    else "UNKNOWN",
                                    "YES"
                                    if px4_ground_now
                                    else "NO"
                                )
                            )

                            last_print = now_print

                        rospy.sleep(0.05)


                    # Do NOT force-disarm.
                    # At this point the aircraft is already physically
                    # on the ground and POSCTL is the pilot fallback.
                    print()
                    print(
                        YELLOW +
                        "[ WAIT ] PX4 has not auto-disarmed yet. "
                        "Use CH5 DISARM while the vehicle remains "
                        "physically on the ground."
                        + RESET
                    )


                    # Keep the process alive instead of declaring mission
                    # failure. Completion occurs as soon as CH5/PX4 disarms.
                    while not rospy.is_shutdown():

                        if (
                            self.state is not None
                            and not self.state.armed
                        ):

                            print(
                                GREEN +
                                "[ OK ] VEHICLE DISARMED - LANDING COMPLETE"
                                + RESET
                            )

                            return 0

                        rospy.sleep(0.10)

            else:

                touchdown_since = None

            if not self.state.armed:

                print()

                print(
                    GREEN +
                    "Vehicle auto-disarmed during landing."
                    + RESET
                )

                return 0

            if self.state.mode != "OFFBOARD":

                print()

                print(
                    RED +
                    "[ABORT] PX4 left OFFBOARD unexpectedly -> %s"
                    % self.state.mode
                    + RESET
                )

                return 1

            if self.point_status == "LANDED_HOLD":

                print()

                print(
                    GREEN +
                    "[ OK ] LAND trajectory complete, z=%.3f"
                    % self.z()
                    + RESET
                )

                break

            if now > deadline:

                print()

                print(
                    RED +
                    "[ABORT] landing trajectory timeout, z=%.3f"
                    % self.z()
                    + RESET
                )

                return 1

            rospy.sleep(0.02)

        # Stop P/V/A trajectory commands before returning to POSCTL.
        self.call(
            "disable_trajectory",
            require=False
        )

        rospy.sleep(0.10)

        print(
            CYAN +
            "[3] Requesting POSCTL for final ground/disarm phase..."
            + RESET
        )

        self.call(
            "request_posctl",
            require=False
        )

        deadline = time.time() + 4.0

        while (
            not rospy.is_shutdown()
            and time.time() < deadline
        ):

            print(
                "\r\033[2K"
                "GROUND   z=%6.3f  armed=%s  mode=%s"
                %
                (
                    self.z(),
                    self.state.armed,
                    self.state.mode
                ),
                end="",
                flush=True
            )

            if not self.state.armed:

                print()

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                print(
                    GREEN +
                    " LANDING COMPLETE - DISARMED"
                    + RESET
                )

                print(
                    GREEN +
                    "========================================"
                    + RESET
                )

                return 0

            rospy.sleep(0.10)

        print()

        print(
            YELLOW +
            "[WARN] Aircraft reached landing state but remains armed."
            + RESET
        )

        print(
            YELLOW +
            "If physically on the ground, disarm with CH5."
            + RESET
        )

        return 0


def main():

    if len(sys.argv) < 2:

        print(
            "Usage: flight_ops_test.py takeoff|land"
        )

        return 1

    command = sys.argv[1].lower()

    rospy.init_node(
        "flight_ops_test",
        anonymous=True
    )

    node = FlightOpsTest()

    if not node.wait_topics():

        print(
            RED +
            "[ABORT] required topics did not become ready"
            + RESET
        )

        return 1

    if command == "takeoff":
        return node.takeoff()

    if command == "land":
        return node.land()

    print(
        "Unknown command: %s"
        % command
    )

    return 1


if __name__ == "__main__":
    sys.exit(main())
