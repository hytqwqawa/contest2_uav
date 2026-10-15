#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <mavros_msgs/PositionTarget.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <string>
#include <sstream>

class OffboardHold
{
public:
    OffboardHold()
        : nh_(),
          pnh_("~"),
          odom_received_(false),
          health_received_(false),
          flight_healthy_(false),
          hold_captured_(false),
          stream_ready_(false)
    {
        pnh_.param<std::string>(
            "odom_topic",
            odom_topic_,
            "/uav1/mavros/local_position/odom");

        pnh_.param<std::string>(
            "health_topic",
            health_topic_,
            "/ego_uav_interface/flight_healthy");

        pnh_.param<std::string>(
            "setpoint_topic",
            setpoint_topic_,
            "/uav1/mavros/setpoint_raw/local");

        pnh_.param(
            "odom_timeout",
            odom_timeout_,
            0.20);

        pnh_.param(
            "publish_rate",
            publish_rate_,
            30.0);

        pnh_.param(
            "stream_ready_time",
            stream_ready_time_,
            1.50);

        odom_sub_ = nh_.subscribe(
            odom_topic_,
            20,
            &OffboardHold::odomCallback,
            this);

        health_sub_ = nh_.subscribe(
            health_topic_,
            20,
            &OffboardHold::healthCallback,
            this);

        setpoint_pub_ =
            nh_.advertise<mavros_msgs::PositionTarget>(
                setpoint_topic_,
                10);

        stream_ready_pub_ =
            nh_.advertise<std_msgs::Bool>(
                "/ego_uav_interface/offboard_stream_ready",
                1,
                true);

        status_pub_ =
            nh_.advertise<std_msgs::String>(
                "/ego_uav_interface/offboard_stream_status",
                1,
                true);

        timer_ = nh_.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &OffboardHold::timerCallback,
            this);

        ROS_INFO("======================================");
        ROS_INFO(" ego_uav_interface offboard hold");
        ROS_INFO("--------------------------------------");
        ROS_INFO(" Odom topic      : %s", odom_topic_.c_str());
        ROS_INFO(" Health topic    : %s", health_topic_.c_str());
        ROS_INFO(" Setpoint topic  : %s", setpoint_topic_.c_str());
        ROS_INFO(" Publish rate    : %.1f Hz", publish_rate_);
        ROS_INFO(" Odom timeout    : %.2f s", odom_timeout_);
        ROS_INFO(" Ready after     : %.2f s", stream_ready_time_);
        ROS_INFO("--------------------------------------");
        ROS_INFO(" IMPORTANT:");
        ROS_INFO(" This node DOES NOT arm the vehicle.");
        ROS_INFO(" This node DOES NOT switch to OFFBOARD.");
        ROS_INFO("======================================");
    }

private:

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_ = *msg;
        odom_time_ = ros::Time::now();
        odom_received_ = true;
    }

    void healthCallback(
        const std_msgs::Bool::ConstPtr& msg)
    {
        flight_healthy_ = msg->data;
        health_time_ = ros::Time::now();
        health_received_ = true;
    }

    void captureHoldPoint()
    {
        hold_x_ = odom_.pose.pose.position.x;
        hold_y_ = odom_.pose.pose.position.y;
        hold_z_ = odom_.pose.pose.position.z;

        hold_yaw_ =
            tf2::getYaw(
                odom_.pose.pose.orientation);

        hold_start_time_ = ros::Time::now();

        hold_captured_ = true;
        stream_ready_ = false;

        ROS_WARN(
            "HOLD POINT CAPTURED: "
            "x=%.3f y=%.3f z=%.3f yaw=%.2f deg",
            hold_x_,
            hold_y_,
            hold_z_,
            hold_yaw_ * 180.0 / M_PI);
    }

    void publishSetpoint()
    {
        mavros_msgs::PositionTarget cmd;

        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";

        /*
         * MAVROS accepts ROS ENU values here and performs
         * the MAVLink frame conversion internally.
         *
         * We intentionally use the same local raw setpoint
         * interface previously used by the working Sunray stack.
         */
        cmd.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        /*
         * Position + yaw control only.
         *
         * Ignore:
         * velocity
         * acceleration
         * yaw rate
         */
        cmd.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        cmd.position.x = hold_x_;
        cmd.position.y = hold_y_;
        cmd.position.z = hold_z_;

        cmd.yaw = hold_yaw_;

        setpoint_pub_.publish(cmd);
    }

    void publishStatus(
        const std::string& text)
    {
        std_msgs::String msg;
        msg.data = text;
        status_pub_.publish(msg);

        std_msgs::Bool ready_msg;
        ready_msg.data = stream_ready_;
        stream_ready_pub_.publish(ready_msg);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();

        bool odom_fresh = false;

        if (odom_received_)
        {
            odom_fresh =
                (now - odom_time_).toSec()
                < odom_timeout_;
        }

        /*
         * flight_healthy is already published at high rate
         * by our preflight monitor stack.
         */
        const bool health_ok =
            health_received_ &&
            flight_healthy_;

        if (!odom_fresh)
        {
            hold_captured_ = false;
            stream_ready_ = false;

            publishStatus("WAIT_ODOM");

            ROS_WARN_THROTTLE(
                1.0,
                "[OFFBOARD STREAM] WAIT_ODOM");

            return;
        }

        if (!health_ok)
        {
            hold_captured_ = false;
            stream_ready_ = false;

            publishStatus("WAIT_FLIGHT_HEALTHY");

            ROS_WARN_THROTTLE(
                1.0,
                "[OFFBOARD STREAM] WAIT_FLIGHT_HEALTHY");

            return;
        }

        if (!hold_captured_)
        {
            captureHoldPoint();
        }

        publishSetpoint();

        const double streaming_time =
            (now - hold_start_time_).toSec();

        if (streaming_time >= stream_ready_time_)
        {
            stream_ready_ = true;
        }

        if (stream_ready_)
        {
            publishStatus("OFFBOARD_STREAM_READY");
        }
        else
        {
            publishStatus("STREAMING_SETPOINTS");
        }

        ROS_INFO_THROTTLE(
            1.0,
            "[OFFBOARD STREAM] "
            "hold=[%.3f %.3f %.3f] "
            "yaw=%.2f deg "
            "stream_time=%.2f s "
            "READY=%s",
            hold_x_,
            hold_y_,
            hold_z_,
            hold_yaw_ * 180.0 / M_PI,
            streaming_time,
            stream_ready_ ? "YES" : "NO");
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber odom_sub_;
    ros::Subscriber health_sub_;

    ros::Publisher setpoint_pub_;
    ros::Publisher stream_ready_pub_;
    ros::Publisher status_pub_;

    ros::Timer timer_;

    std::string odom_topic_;
    std::string health_topic_;
    std::string setpoint_topic_;

    double odom_timeout_;
    double publish_rate_;
    double stream_ready_time_;

    nav_msgs::Odometry odom_;

    ros::Time odom_time_;
    ros::Time health_time_;
    ros::Time hold_start_time_;

    bool odom_received_;
    bool health_received_;
    bool flight_healthy_;

    bool hold_captured_;
    bool stream_ready_;

    double hold_x_;
    double hold_y_;
    double hold_z_;
    double hold_yaw_;
};

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "offboard_hold_node");

    OffboardHold node;

    ros::spin();

    return 0;
}
