#!/usr/bin/env python3
import importlib.util
import math
import time
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parent


def load_align():
    p = ROOT / "uav2_tangent_align_only.py"
    spec = importlib.util.spec_from_file_location(
        "align_impl", str(p)
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.AlignOnly


def runtime_ok(n, where):
    if (
        n.state is None
        or not n.state.armed
        or n.state.mode != "OFFBOARD"
        or not n.health
        or not n.mission
        or not n.demo_safe
    ):
        raise RuntimeError(
            "runtime safety lost %s" % where
        )

    if n.mux_status != "EGO_ACTIVE":
        raise RuntimeError(
            "MUX left EGO_ACTIVE %s: %s"
            % (where, n.mux_status)
        )


def main():

    rospy.init_node(
        "uav2_tangent_align_execute"
    )

    AlignOnly = load_align()
    n = AlignOnly()

    # Keep the proven ALIGN implementation unchanged.
    # Only suppress its normal post-ALIGN return to POINT.
    restore = n.restore_point_hold

    def suppress_restore():
        rospy.logwarn(
            "COMBINED MODE: "
            "keep EGO_ACTIVE + FIXED after ALIGN"
        )

    n.restore_point_hold = suppress_restore

    trigger_pub = rospy.Publisher(
        "/contest2/traj_start_trigger",
        PoseStamped,
        queue_size=1
    )

    try:

        # ==================================================
        # 1. PROVEN ALIGN-ONLY IMPLEMENTATION
        # ==================================================

        n.run()

        # From here on, failures restore a REAL POINT hold.
        n.restore_point_hold = restore

        runtime_ok(
            n,
            "after ALIGN"
        )

        with open(
            n.target_file,
            "r"
        ) as f:
            target = yaml.safe_load(f)

        traj_id = int(
            target["traj_id"]
        )

        if (
            n.cmd is None
            or int(n.cmd.trajectory_id) != traj_id
        ):
            raise RuntimeError(
                "trajectory changed before trigger"
            )

        # ==================================================
        # PRE-TRIGGER SETTLE GATE
        #
        # ALIGN may finish during a normal hover excursion.
        # Keep EGO_ACTIVE + FIXED and aligned yaw until the
        # SAME frozen trajectory's first gap is safely back
        # below 0.08 m for a continuous interval.
        #
        # The 0.08-m safety threshold is NOT relaxed.
        # ==================================================

        yaw_target_deg = float(
            target["execution_initial_yaw_deg"]
        )

        settle_required = 0.40
        settle_since = None

        deadline = time.time() + 8.0

        ready_to_trigger = False

        print()
        print("PRE-TRIGGER SETTLE:")
        print(
            "  require first gap < 0.08 m "
            "for %.2f s"
            % settle_required
        )

        while time.time() < deadline:

            runtime_ok(
                n,
                "during pre-trigger settle"
            )

            if n.cmd is None:
                raise RuntimeError(
                    "PositionCommand unavailable "
                    "during pre-trigger settle"
                )

            if int(n.cmd.trajectory_id) != traj_id:
                raise RuntimeError(
                    "trajectory changed "
                    "during pre-trigger settle"
                )

            p = n.odom.pose.pose.position

            gap_xy = math.hypot(
                n.cmd.position.x - p.x,
                n.cmd.position.y - p.y
            )

            gap_z = abs(
                n.cmd.position.z - p.z
            )

            yaw = n.current_yaw()

            yaw_err = math.degrees(
                math.atan2(
                    math.sin(
                        math.radians(yaw_target_deg) - yaw
                    ),
                    math.cos(
                        math.radians(yaw_target_deg) - yaw
                    )
                )
            )

            good = (
                gap_xy < 0.08
                and gap_z < 0.08
                and abs(yaw_err) <= 8.0
            )

            if good:

                if settle_since is None:
                    settle_since = time.time()

                stable = (
                    time.time()
                    -
                    settle_since
                )

            else:

                settle_since = None
                stable = 0.0

            print(
                "\rPRETRIG "
                "dXY=%.3f "
                "dZ=%.3f "
                "yaw_err=%+.2f "
                "stable=%.2f/%.2f"
                %
                (
                    gap_xy,
                    gap_z,
                    yaw_err,
                    stable,
                    settle_required
                ),
                end="",
                flush=True
            )

            if stable >= settle_required:

                ready_to_trigger = True
                break

            rospy.sleep(0.05)

        print()

        if not ready_to_trigger:
            raise RuntimeError(
                "pre-trigger settle timeout; "
                "first gap never became safely < 0.08 m"
            )

        print(
            "[ OK ] pre-trigger settle: "
            "dXY %.3f m  dZ %.3f m"
            %
            (
                gap_xy,
                gap_z
            )
        )

        deadline = time.time() + 2.0

        while (
            time.time() < deadline
            and trigger_pub.get_num_connections() < 1
        ):
            rospy.sleep(0.02)

        if trigger_pub.get_num_connections() < 1:
            raise RuntimeError(
                "/contest2/traj_start_trigger has no subscriber"
            )

        frozen = (
            n.cmd.position.x,
            n.cmd.position.y,
            n.cmd.position.z
        )

        trig = PoseStamped()

        trig.header.stamp = rospy.Time.now()
        trig.header.frame_id = "world"

        trig.pose.position.x = p.x
        trig.pose.position.y = p.y
        trig.pose.position.z = p.z
        trig.pose.orientation.w = 1.0

        print()
        print("========================================")
        print(
            " EXECUTION TRIGGER  traj_id=%d"
            % traj_id
        )
        print(
            " FIXED XYZ retained until "
            "clock release is verified"
        )
        print("========================================")

        # EXACTLY ONE trigger.
        # Repeated triggers would reset t=0 repeatedly.
        trigger_pub.publish(trig)

        # ==================================================
        # 2. VERIFY TRIGGER WHILE XYZ IS STILL FIXED
        # ==================================================

        released = False

        deadline = time.time() + 2.0

        while time.time() < deadline:

            runtime_ok(
                n,
                "after trigger"
            )

            if n.cmd is None:
                rospy.sleep(0.02)
                continue

            move = math.sqrt(
                (
                    n.cmd.position.x
                    - frozen[0]
                ) ** 2
                +
                (
                    n.cmd.position.y
                    - frozen[1]
                ) ** 2
                +
                (
                    n.cmd.position.z
                    - frozen[2]
                ) ** 2
            )

            # A new ID also means execution started and
            # EGO immediately issued an in-flight replan.
            if (
                move > 0.003
                or int(n.cmd.trajectory_id) != traj_id
            ):

                p = n.odom.pose.pose.position

                gap_xy = math.hypot(
                    n.cmd.position.x - p.x,
                    n.cmd.position.y - p.y
                )

                gap_z = abs(
                    n.cmd.position.z - p.z
                )

                if (
                    gap_xy > 0.08
                    or gap_z > 0.08
                ):
                    raise RuntimeError(
                        "post-trigger release gap "
                        "too large: "
                        "dXY=%.3f dZ=%.3f"
                        %
                        (
                            gap_xy,
                            gap_z
                        )
                    )

                released = True

                print(
                    "[ OK ] clock released: "
                    "cmd_move=%.4f "
                    "dXY=%.3f dZ=%.3f"
                    %
                    (
                        move,
                        gap_xy,
                        gap_z
                    )
                )

                break

            rospy.sleep(0.02)

        if not released:
            raise RuntimeError(
                "trigger not observed; "
                "PositionCommand remained frozen"
            )

        # Only AFTER trigger acceptance is proven
        # do we release XYZ translation.
        n.yaw_mode_pub.publish(
            String(data="TANGENT")
        )

        print(
            "[ OK ] yaw mode -> TANGENT; "
            "translation released"
        )

        # ==================================================
        # 3. 4-M EXECUTION MONITOR
        # ==================================================

        with open(
            ROOT / "session_frame.yaml",
            "r"
        ) as f:
            session = yaml.safe_load(f)

        goal = session["goal_world"]

        gx = float(goal["x"])
        gy = float(goal["y"])
        gz = float(goal["z"])

        print()
        print("========================================")
        print(
            " 5M EXECUTION -> "
            "goal=(%.3f, %.3f, %.3f)"
            %
            (
                gx,
                gy,
                gz
            )
        )
        print(
            " CH7 LOW / CH6 takeover remains available"
        )
        print("========================================")

        stable_since = None

        last_id = int(
            n.cmd.trajectory_id
        )

        deadline = time.time() + 35.0

        rate = rospy.Rate(20)

        while time.time() < deadline:

            runtime_ok(
                n,
                "during execution"
            )

            if n.cmd is None:
                raise RuntimeError(
                    "PositionCommand unavailable"
                )

            tid = int(
                n.cmd.trajectory_id
            )

            if tid != last_id:

                print()

                print(
                    "[ REPLAN ] traj_id %d -> %d"
                    %
                    (
                        last_id,
                        tid
                    )
                )

                last_id = tid

            p = n.odom.pose.pose.position
            v = n.odom.twist.twist.linear

            dxy = math.hypot(
                p.x - gx,
                p.y - gy
            )

            dz = abs(
                p.z - gz
            )

            speed = math.sqrt(
                v.x * v.x
                +
                v.y * v.y
                +
                v.z * v.z
            )

            if (
                dxy <= 0.15
                and dz <= 0.15
                and speed <= 0.15
            ):

                if stable_since is None:
                    stable_since = time.time()

                stable = (
                    time.time()
                    -
                    stable_since
                )

            else:

                stable_since = None
                stable = 0.0

            print(
                "\rEXEC "
                "traj=%d "
                "dXY=%.3f "
                "dZ=%.3f "
                "speed=%.3f "
                "stable=%.2f/0.80"
                %
                (
                    tid,
                    dxy,
                    dz,
                    speed,
                    stable
                ),
                end="",
                flush=True
            )

            if stable >= 0.80:

                print()

                # Capture current final yaw, then POINT
                # captures CURRENT final XYZ hold.
                n.yaw_mode_pub.publish(
                    String(data="HOLD")
                )

                restore()

                print(
                    "========================================"
                )
                print(
                    " FULL ALIGN + EXECUTE PASSED"
                )
                print(
                    " final goal dXY = %.3f m"
                    % dxy
                )
                print(
                    " [ OK ] POINT HOLD captured "
                    "at final position"
                )
                print(
                    "========================================"
                )

                return

            rate.sleep()

        raise RuntimeError(
            "5-m execution timeout"
        )

    except Exception as e:

        n.restore_point_hold = restore

        rospy.logerr(
            "ALIGN+EXECUTE FAILED: %s",
            str(e)
        )

        try:
            restore()
        except Exception:
            pass

        raise


if __name__ == "__main__":
    main()
