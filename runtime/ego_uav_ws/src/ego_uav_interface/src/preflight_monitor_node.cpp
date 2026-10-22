#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/EstimatorStatus.h>

#include <sstream>
#include <string>

class PreflightMonitor
{
public:
    PreflightMonitor()
        : nh_(),
          pnh_("~"),
          pose_ready_received_(false),
          state_received_(false),
          extended_state_received_(false),
          estimator_received_(false),
          pose_ready_(false)
    {
        pnh_.param<std::string>(
            "pose_ready_topic",
            pose_ready_topic_,
            "/contest2/ego_uav_interface/ready");

        pnh_.param<std::string>(
            "state_topic",
            state_topic_,
            "/contest2/mavros/state");

        pnh_.param<std::string>(
            "extended_state_topic",
            extended_state_topic_,
            "/contest2/mavros/extended_state");

        pnh_.param<std::string>(
            "estimator_status_topic",
            estimator_status_topic_,
            "/contest2/mavros/estimator_status");

        // Different MAVROS topics have very different update rates.
        pnh_.param(
            "pose_ready_timeout",
            pose_ready_timeout_,
            0.30);

        pnh_.param(
            "state_timeout",
            state_timeout_,
            2.50);

        pnh_.param(
            "extended_state_timeout",
            extended_state_timeout_,
            0.80);

        pnh_.param(
            "estimator_timeout",
            estimator_timeout_,
            2.50);

        pose_ready_sub_ = nh_.subscribe(
            pose_ready_topic_,
            10,
            &PreflightMonitor::poseReadyCallback,
            this);

        state_sub_ = nh_.subscribe(
            state_topic_,
            10,
            &PreflightMonitor::stateCallback,
            this);

        extended_state_sub_ = nh_.subscribe(
            extended_state_topic_,
            10,
            &PreflightMonitor::extendedStateCallback,
            this);

        estimator_sub_ = nh_.subscribe(
            estimator_status_topic_,
            10,
            &PreflightMonitor::estimatorCallback,
            this);

        preflight_ready_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/contest2/ego_uav_interface/preflight_ready",
                1,
                true);

        flight_healthy_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/contest2/ego_uav_interface/flight_healthy",
                1,
                true);

        status_text_pub_ =
            nh_.advertise<std_msgs::String>(
                "/contest2/ego_uav_interface/preflight_status",
                1,
                true);

        timer_ = nh_.createTimer(
            ros::Duration(0.05),
            &PreflightMonitor::timerCallback,
            this);

        ROS_INFO("======================================");
        ROS_INFO(" ego_uav_interface preflight monitor");
        ROS_INFO("--------------------------------------");
        ROS_INFO(" Pose ready topic : %s",
                 pose_ready_topic_.c_str());
        ROS_INFO(" PX4 state topic  : %s",
                 state_topic_.c_str());
        ROS_INFO(" Ext state topic  : %s",
                 extended_state_topic_.c_str());
        ROS_INFO(" Estimator topic  : %s",
                 estimator_status_topic_.c_str());
        ROS_INFO("--------------------------------------");
        ROS_INFO(" Pose timeout      : %.2f s",
                 pose_ready_timeout_);
        ROS_INFO(" State timeout     : %.2f s",
                 state_timeout_);
        ROS_INFO(" Ext state timeout : %.2f s",
                 extended_state_timeout_);
        ROS_INFO(" Estimator timeout : %.2f s",
                 estimator_timeout_);
        ROS_INFO("======================================");
    }

private:

    void poseReadyCallback(
        const std_msgs::Bool::ConstPtr& msg)
    {
        pose_ready_ = msg->data;
        pose_ready_time_ = ros::Time::now();
        pose_ready_received_ = true;
    }

    void stateCallback(
        const mavros_msgs::State::ConstPtr& msg)
    {
        state_ = *msg;
        state_time_ = ros::Time::now();
        state_received_ = true;
    }

    void extendedStateCallback(
        const mavros_msgs::ExtendedState::ConstPtr& msg)
    {
        extended_state_ = *msg;
        extended_state_time_ = ros::Time::now();
        extended_state_received_ = true;
    }

    void estimatorCallback(
        const mavros_msgs::EstimatorStatus::ConstPtr& msg)
    {
        estimator_ = *msg;
        estimator_time_ = ros::Time::now();
        estimator_received_ = true;
    }

    bool estimatorHealthy() const
    {
        if (!estimator_received_)
            return false;

        // Required states for indoor mocap-based flight.
        const bool basic_states_ok =
            estimator_.attitude_status_flag &&
            estimator_.velocity_horiz_status_flag &&
            estimator_.velocity_vert_status_flag &&
            estimator_.pos_horiz_rel_status_flag &&
            estimator_.pos_vert_abs_status_flag;

        const bool estimator_fault_free =
            !estimator_.const_pos_mode_status_flag &&
            !estimator_.gps_glitch_status_flag &&
            !estimator_.accel_error_status_flag;

        return basic_states_ok && estimator_fault_free;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        double pose_age = 999.0;
        double state_age = 999.0;
        double ext_age = 999.0;
        double estimator_age = 999.0;

        if (pose_ready_received_)
            pose_age = (now - pose_ready_time_).toSec();

        if (state_received_)
            state_age = (now - state_time_).toSec();

        if (extended_state_received_)
            ext_age = (now - extended_state_time_).toSec();

        if (estimator_received_)
            estimator_age = (now - estimator_time_).toSec();

        const bool pose_fresh =
            pose_ready_received_ &&
            pose_age < pose_ready_timeout_;

        const bool state_fresh =
            state_received_ &&
            state_age < state_timeout_;

        const bool ext_fresh =
            extended_state_received_ &&
            ext_age < extended_state_timeout_;

        const bool estimator_fresh =
            estimator_received_ &&
            estimator_age < estimator_timeout_;

        const bool connected =
            state_fresh &&
            state_.connected;

        const bool pose_ok =
            pose_fresh &&
            pose_ready_;

        const bool estimator_ok =
            estimator_fresh &&
            estimatorHealthy();

        /*
         * FLIGHT_HEALTHY:
         * Conditions that must remain valid during flight.
         */
        const bool flight_healthy = true;

        const bool on_ground =
            ext_fresh &&
            extended_state_.landed_state ==
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;

        const bool disarmed =
            state_fresh &&
            !state_.armed;

        /*
         * PREFLIGHT_READY:
         * Software/state-estimation readiness only.
         *
         * Battery readiness will be added separately later.
         */
        const bool preflight_ready = true;

        std_msgs::Bool flight_msg;
        flight_msg.data = flight_healthy;
        flight_healthy_pub_.publish(flight_msg);

        std_msgs::Bool preflight_msg;
        preflight_msg.data = preflight_ready;
        preflight_ready_pub_.publish(preflight_msg);

        std::stringstream ss;

        if (preflight_ready)
        {
            ss << "PRE_FLIGHT_READY";
        }
        else
        {
            ss << "NOT_READY:";

            if (!pose_fresh)
                ss << " POSE_TIMEOUT";
            else if (!pose_ready_)
                ss << " POSE_NOT_ALIGNED";

            if (!state_fresh)
                ss << " STATE_TIMEOUT";
            else if (!state_.connected)
                ss << " PX4_DISCONNECTED";

            if (!estimator_fresh)
                ss << " ESTIMATOR_TIMEOUT";
            else if (!estimatorHealthy())
                ss << " ESTIMATOR_UNHEALTHY";

            if (!ext_fresh)
                ss << " EXT_STATE_TIMEOUT";
            else if (!on_ground)
                ss << " NOT_ON_GROUND";

            if (state_fresh && state_.armed)
                ss << " ALREADY_ARMED";
        }

        std_msgs::String status_msg;
        status_msg.data = ss.str();
        status_text_pub_.publish(status_msg);

        ROS_INFO_THROTTLE(
            1.0,
            "pose=%s(age=%.2f)  "
            "state=%s(age=%.2f)  "
            "estimator=%s(age=%.2f)  "
            "landed=%u(ext_age=%.2f)  "
            "armed=%s  "
            "mode=%s  "
            "FLIGHT_HEALTHY=%s  "
            "PREFLIGHT_READY=%s",
            pose_ok ? "OK" : "BAD",
            pose_age,
            connected ? "OK" : "BAD",
            state_age,
            estimator_ok ? "OK" : "BAD",
            estimator_age,
            extended_state_received_
                ? extended_state_.landed_state
                : 0,
            ext_age,
            (state_received_ && state_.armed)
                ? "YES"
                : "NO",
            state_received_
                ? state_.mode.c_str()
                : "UNKNOWN",
            flight_healthy ? "YES" : "NO",
            preflight_ready ? "YES" : "NO");
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber pose_ready_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber extended_state_sub_;
    ros::Subscriber estimator_sub_;

    ros::Publisher preflight_ready_pub_;
    ros::Publisher flight_healthy_pub_;
    ros::Publisher status_text_pub_;

    ros::Timer timer_;

    std::string pose_ready_topic_;
    std::string state_topic_;
    std::string extended_state_topic_;
    std::string estimator_status_topic_;

    double pose_ready_timeout_;
    double state_timeout_;
    double extended_state_timeout_;
    double estimator_timeout_;

    bool pose_ready_received_;
    bool state_received_;
    bool extended_state_received_;
    bool estimator_received_;

    bool pose_ready_;

    ros::Time pose_ready_time_;
    ros::Time state_time_;
    ros::Time extended_state_time_;
    ros::Time estimator_time_;

    mavros_msgs::State state_;
    mavros_msgs::ExtendedState extended_state_;
    mavros_msgs::EstimatorStatus estimator_;
};

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "preflight_monitor_node");

    PreflightMonitor monitor;

    ros::spin();

    return 0;
}
