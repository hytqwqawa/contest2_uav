#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <cmath>
#include <string>

class StateMonitor
{
public:
    StateMonitor()
        : nh_(),
          pnh_("~"),
          mocap_received_(false),
          odom_received_(false),
          range_received_(false),
          ready_(false)
    {
        pnh_.param<std::string>(
            "mocap_pose_topic",
            mocap_pose_topic_,
            "/contest2/ego_uav_interface/external_pose_ref");

        pnh_.param<std::string>(
            "px4_odom_topic",
            px4_odom_topic_,
            "/contest2/mavros/local_position/odom");

        pnh_.param(
            "position_error_max",
            position_error_max_,
            0.10);

        pnh_.param(
            "yaw_error_max_deg",
            yaw_error_max_deg_,
            5.0);

        pnh_.param(
            "message_timeout",
            message_timeout_,
            0.15);

        pnh_.param(
            "stable_time",
            stable_time_,
            1.5);

        // READY entry remains strict, but an already healthy
        // system is not dropped by a single transient mismatch.
        pnh_.param(
            "ready_loss_time",
            ready_loss_time_,
            0.40);

        // MOTION_AWARE_XY_V1
        //
        // Keep the original strict static consistency test.
        // During horizontal motion, PX4 EKF odometry can lag
        // external vision slightly. Allow only the bounded
        // motion-correlated part of this XY disagreement.
        //
        // This modifies only the SOFT retention test.
        // Existing hard-fault thresholds remain unchanged.
        pnh_.param(
            "motion_xy_lag_allowance",
            motion_xy_lag_allowance_,
            0.25);

        pnh_.param(
            "motion_xy_soft_cap",
            motion_xy_soft_cap_,
            0.20);

        // Severe disagreement still causes an immediate drop.
        pnh_.param(
            "hard_horizontal_error_max",
            hard_horizontal_error_max_,
            0.30);

        pnh_.param(
            "hard_z_error_max",
            hard_z_error_max_,
            0.50);

        pnh_.param(
            "hard_yaw_error_max_deg",
            hard_yaw_error_max_deg_,
            15.0);

        // UAV2 near-ground adaptation.
        //
        // TFMini changes rapidly near touchdown while PX4's
        // filtered vertical state can lag. Horizontal position
        // and yaw remain strictly checked.
        pnh_.param<std::string>(
            "range_topic",
            range_topic_,
            "/contest2/mavros/distance_sensor/hrlv_ez4_pub");

        pnh_.param(
            "near_ground_range",
            near_ground_range_,
            0.25);

        pnh_.param(
            "near_ground_z_error_max",
            near_ground_z_error_max_,
            0.25);

        pnh_.param(
            "range_timeout",
            range_timeout_,
            0.30);

        mocap_sub_ = nh_.subscribe(
            mocap_pose_topic_,
            20,
            &StateMonitor::mocapCallback,
            this);

        odom_sub_ = nh_.subscribe(
            px4_odom_topic_,
            20,
            &StateMonitor::odomCallback,
            this);

        range_sub_ = nh_.subscribe(
            range_topic_,
            20,
            &StateMonitor::rangeCallback,
            this);

        ready_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/contest2/ego_uav_interface/ready",
                1,
                true);

        pos_error_pub_ =
            nh_.advertise<geometry_msgs::Vector3Stamped>(
                "/contest2/ego_uav_interface/position_error",
                10);

        yaw_error_pub_ =
            nh_.advertise<std_msgs::Float64>(
                "/contest2/ego_uav_interface/yaw_error_deg",
                10);

        timer_ = nh_.createTimer(
            ros::Duration(0.05),
            &StateMonitor::timerCallback,
            this);

        ROS_INFO("======================================");
        ROS_INFO(" ego_uav_interface state monitor");
        ROS_INFO("--------------------------------------");
        ROS_INFO(" Mocap topic : %s",
                 mocap_pose_topic_.c_str());
        ROS_INFO(" PX4 odom    : %s",
                 px4_odom_topic_.c_str());
        ROS_INFO(" Pos limit   : %.3f m",
                 position_error_max_);
        ROS_INFO(" Yaw limit   : %.2f deg",
                 yaw_error_max_deg_);
        ROS_INFO(" Timeout     : %.3f s",
                 message_timeout_);
        ROS_INFO(" Stable time : %.2f s",
                 stable_time_);
        ROS_INFO(" READY loss persistence : %.2f s",
                 ready_loss_time_);
        ROS_INFO(" Motion XY lag allowance : %.2f s",
                 motion_xy_lag_allowance_);
        ROS_INFO(" Motion XY soft cap : %.2f m",
                 motion_xy_soft_cap_);
        ROS_INFO(" Hard XY limit : %.2f m",
                 hard_horizontal_error_max_);
        ROS_INFO(" Hard Z limit : %.2f m",
                 hard_z_error_max_);
        ROS_INFO(" Hard yaw limit : %.1f deg",
                 hard_yaw_error_max_deg_);
        ROS_INFO(" Range topic : %s",
                 range_topic_.c_str());
        ROS_INFO(" Near ground : <= %.2f m",
                 near_ground_range_);
        ROS_INFO(" Near-ground Z limit : %.2f m",
                 near_ground_z_error_max_);
        ROS_INFO("======================================");
    }

private:

    double wrapPi(double angle)
    {
        return std::atan2(
            std::sin(angle),
            std::cos(angle));
    }

    void mocapCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        mocap_pose_ = *msg;
        mocap_receive_time_ = ros::Time::now();
        mocap_received_ = true;
    }

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
    {
        px4_odom_ = *msg;
        odom_receive_time_ = ros::Time::now();
        odom_received_ = true;
    }

    void rangeCallback(
        const sensor_msgs::Range::ConstPtr& msg)
    {
        if (std::isfinite(msg->range) &&
            msg->range > 0.0)
        {
            range_m_ = msg->range;
            range_receive_time_ = ros::Time::now();
            range_received_ = true;
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        if (!mocap_received_ || !odom_received_)
        {
            ready_ = false;
            stable_start_ = ros::Time(0);
            bad_start_ = ros::Time(0);

            publishReady();

            ROS_WARN_THROTTLE(
                1.0,
                "[WAIT] mocap=%d odom=%d",
                mocap_received_,
                odom_received_);

            return;
        }

        const double mocap_age =
            (now - mocap_receive_time_).toSec();

        const double odom_age =
            (now - odom_receive_time_).toSec();

        const bool mocap_fresh =
            mocap_age < message_timeout_;

        const bool odom_fresh =
            odom_age < message_timeout_;

        // Position error
        const double ex =
            mocap_pose_.pose.position.x
            - px4_odom_.pose.pose.position.x;

        const double ey =
            mocap_pose_.pose.position.y
            - px4_odom_.pose.pose.position.y;

        const double ez =
            mocap_pose_.pose.position.z
            - px4_odom_.pose.pose.position.z;

        const double position_error =
            std::sqrt(
                ex * ex +
                ey * ey +
                ez * ez);

        // Yaw error
        tf2::Quaternion q_mocap;
        tf2::Quaternion q_px4;

        tf2::fromMsg(
            mocap_pose_.pose.orientation,
            q_mocap);

        tf2::fromMsg(
            px4_odom_.pose.pose.orientation,
            q_px4);

        const double yaw_mocap =
            tf2::getYaw(q_mocap);

        const double yaw_px4 =
            tf2::getYaw(q_px4);

        const double yaw_error =
            wrapPi(yaw_mocap - yaw_px4);

        const double yaw_error_deg =
            yaw_error * 180.0 / M_PI;

        // Publish errors
        geometry_msgs::Vector3Stamped pos_error_msg;
        pos_error_msg.header.stamp = now;
        pos_error_msg.header.frame_id = "map";

        pos_error_msg.vector.x = ex;
        pos_error_msg.vector.y = ey;
        pos_error_msg.vector.z = ez;

        pos_error_pub_.publish(pos_error_msg);

        std_msgs::Float64 yaw_msg;
        yaw_msg.data = yaw_error_deg;

        yaw_error_pub_.publish(yaw_msg);

        // --------------------------------------------------
        // Instantaneous health
        //
        // Above the near-ground region, retain the original
        // strict 3-D consistency check.
        //
        // Near the floor, TFMini is the external Z source and
        // the PX4 vertical estimate can lag during touchdown.
        // Keep XY and yaw strict, but give Z its own tolerance.
        // --------------------------------------------------

        const double horizontal_error =
            std::sqrt(
                ex * ex +
                ey * ey);

        const double range_age =
            range_received_
            ? (now - range_receive_time_).toSec()
            : 1e9;

        const bool range_fresh =
            range_received_ &&
            range_age < range_timeout_;

        const bool near_ground =
            range_fresh &&
            range_m_ <= near_ground_range_;

        // MOTION_AWARE_XY_V1
        //
        // At zero horizontal velocity:
        //   lag_allowance = 0
        // so the original position_error_max_ requirement
        // is recovered exactly.
        //
        // In flight:
        //   allowed dynamic XY component ~= v_xy * tau.
        //
        // Raw XY disagreement must additionally remain below
        // motion_xy_soft_cap_. The existing 0.30-m hard fault
        // remains independent and unchanged.
        const double vxy =
            std::sqrt(
                px4_odom_.twist.twist.linear.x *
                px4_odom_.twist.twist.linear.x
                +
                px4_odom_.twist.twist.linear.y *
                px4_odom_.twist.twist.linear.y);

        const double lag_allowance =
            motion_xy_lag_allowance_ * vxy;

        const double compensated_xy =
            std::max(
                0.0,
                horizontal_error - lag_allowance);

        const double compensated_position_error =
            std::sqrt(
                compensated_xy * compensated_xy
                +
                ez * ez);

        const bool position_ok =
            near_ground
            ?
            (
                horizontal_error < position_error_max_ &&
                std::fabs(ez) < near_ground_z_error_max_
            )
            :
            (
                horizontal_error < motion_xy_soft_cap_
                &&
                compensated_position_error
                    < position_error_max_
            );

        const bool yaw_ok =
            std::fabs(yaw_error_deg)
            < yaw_error_max_deg_;

        const bool instant_ready =
            mocap_fresh &&
            odom_fresh &&
            position_ok &&
            yaw_ok;

        // --------------------------------------------------
        // READY hysteresis
        //
        // Entry:
        //   retain the original strict requirement and
        //   stable_time_ qualification.
        //
        // Retention:
        //   a normal threshold violation must persist for
        //   ready_loss_time_ before READY is dropped.
        //
        // Hard faults:
        //   stale localization or a large pose disagreement
        //   still drops READY immediately.
        // --------------------------------------------------

        const bool hard_fault =
            !mocap_fresh ||
            !odom_fresh ||
            horizontal_error >= hard_horizontal_error_max_ ||
            std::fabs(ez) >= hard_z_error_max_ ||
            std::fabs(yaw_error_deg)
                >= hard_yaw_error_max_deg_;

        if (!ready_)
        {
            // Strict acquisition logic.
            bad_start_ = ros::Time(0);

            if (instant_ready)
            {
                if (stable_start_.isZero())
                {
                    stable_start_ = now;
                }

                if ((now - stable_start_).toSec()
                    >= stable_time_)
                {
                    ready_ = true;
                    bad_start_ = ros::Time(0);

                    ROS_INFO(
                        "[STATE MONITOR] READY acquired");
                }
            }
            else
            {
                stable_start_ = ros::Time(0);
            }
        }
        else
        {
            // Already READY.
            if (hard_fault)
            {
                ready_ = false;
                stable_start_ = ros::Time(0);
                bad_start_ = ros::Time(0);

                ROS_ERROR(
                    "[STATE MONITOR] HARD fault -> READY lost: "
                    "xy=%.3f ez=%.3f yaw=%.2f "
                    "mocap_fresh=%d odom_fresh=%d",
                    horizontal_error,
                    ez,
                    yaw_error_deg,
                    mocap_fresh,
                    odom_fresh);
            }
            else if (instant_ready)
            {
                // Normal again before persistence timeout.
                bad_start_ = ros::Time(0);
            }
            else
            {
                // Soft inconsistency: start/continue timer.
                if (bad_start_.isZero())
                {
                    bad_start_ = now;

                    ROS_WARN(
                        "[STATE MONITOR] transient mismatch "
                        "started: "
                        "xy=%.3f comp_xy=%.3f "
                        "vxy=%.3f ez=%.3f yaw=%.2f",
                        horizontal_error,
                        compensated_xy,
                        vxy,
                        ez,
                        yaw_error_deg);
                }

                const double bad_age =
                    (now - bad_start_).toSec();

                if (bad_age >= ready_loss_time_)
                {
                    ready_ = false;
                    stable_start_ = ros::Time(0);
                    bad_start_ = ros::Time(0);

                    ROS_ERROR(
                        "[STATE MONITOR] mismatch persisted "
                        "%.2f s -> READY lost",
                        bad_age);
                }
            }
        }

        publishReady();

        ROS_INFO_THROTTLE(
            1.0,
            "pos_err=%.4f m "
            "xy=%.4f comp_xy=%.4f "
            "vxy=%.3f "
            "[%.4f %.4f %.4f]  "
            "yaw_err=%.2f deg  "
            "range=%.3f "
            "near_ground=%s  "
            "bad_age=%.2f  "
            "READY=%s",
            position_error,
            horizontal_error,
            compensated_xy,
            vxy,
            ex,
            ey,
            ez,
            yaw_error_deg,
            range_received_ ? range_m_ : -1.0,
            near_ground ? "YES" : "NO",
            bad_start_.isZero()
                ? 0.0
                : (now - bad_start_).toSec(),
            ready_ ? "YES" : "NO");
    }

    void publishReady()
    {
        std_msgs::Bool msg;
        msg.data = ready_;
        ready_pub_.publish(msg);
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber mocap_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber range_sub_;

    ros::Publisher ready_pub_;
    ros::Publisher pos_error_pub_;
    ros::Publisher yaw_error_pub_;

    ros::Timer timer_;

    std::string mocap_pose_topic_;
    std::string px4_odom_topic_;
    std::string range_topic_;

    double position_error_max_;
    double yaw_error_max_deg_;
    double message_timeout_;
    double stable_time_;

    double ready_loss_time_;
    // MOTION_AWARE_XY_V1
    double motion_xy_lag_allowance_;
    double motion_xy_soft_cap_;

    double hard_horizontal_error_max_;
    double hard_z_error_max_;
    double hard_yaw_error_max_deg_;

    double near_ground_range_;
    double near_ground_z_error_max_;
    double range_timeout_;

    geometry_msgs::PoseStamped mocap_pose_;
    nav_msgs::Odometry px4_odom_;

    ros::Time mocap_receive_time_;
    ros::Time odom_receive_time_;
    ros::Time range_receive_time_;

    ros::Time stable_start_;
    ros::Time bad_start_;

    double range_m_ = 0.0;

    bool mocap_received_;
    bool odom_received_;
    bool range_received_;
    bool ready_;
};


int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "state_monitor_node");

    StateMonitor monitor;

    ros::spin();

    return 0;
}
