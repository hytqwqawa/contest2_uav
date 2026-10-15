#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

bool receive_traj_ = false;

// EXEC_PREVIEW_FREEZE_V3
// A newly planned B-spline is previewed at t=0 until the
// explicit /traj_start_trigger is received.
bool execution_started_ = false;

// EXEC_PREVIEW_GOAL_GATE_V5
//
// A MANUAL goal arms preview freeze for the NEXT B-spline only.
// Normal in-flight EGO replans after execution has started must
// remain executable and must NOT be frozen again.
bool preview_request_pending_ = false;

vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_;

// EXEC_TRIGGER_YAW_SYNC_V1
// Actual aircraft yaw used to seed traj_server yaw state
// exactly when execution of the aligned trajectory begins.
bool have_odom_yaw_ = false;
double odom_yaw_ = 0.0;
ros::Time odom_yaw_rx_time_;

// EXEC_PREVIEW_GOAL_GATE_V5
void previewGoalCallback(
    const geometry_msgs::PoseStampedConstPtr &msg)
{
  preview_request_pending_ = true;

  ROS_WARN(
      "[Traj server] PREVIEW REQUEST: "
      "next B-spline will be frozen at t=0. "
      "goal=(%.3f, %.3f, %.3f)",
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z
  );
}


// EXEC_TRIGGER_YAW_SYNC_V1
void odomCallback(
    const nav_msgs::OdometryConstPtr &msg)
{
  const auto &q = msg->pose.pose.orientation;

  const double siny_cosp =
      2.0 * (q.w * q.z + q.x * q.y);

  const double cosy_cosp =
      1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  odom_yaw_ = atan2(
      siny_cosp,
      cosy_cosp);

  odom_yaw_rx_time_ = ros::Time::now();
  have_odom_yaw_ = true;
}


// TRAJ_START_RESET_V3
void startTriggerCallback(
    const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!receive_traj_)
  {
    ROS_WARN(
        "[Traj server] execution trigger ignored: "
        "no trajectory loaded."
    );
    return;
  }

  const ros::Time now = ros::Time::now();

  if (!have_odom_yaw_)
  {
    ROS_ERROR(
        "[Traj server] execution trigger rejected: "
        "actual odom yaw unavailable."
    );
    return;
  }

  const double odom_age =
      (now - odom_yaw_rx_time_).toSec();

  if (odom_age < 0.0 || odom_age > 0.20)
  {
    ROS_ERROR(
        "[Traj server] execution trigger rejected: "
        "odom yaw stale (age=%.3f s).",
        odom_age
    );
    return;
  }

  // The vehicle has already completed ALIGN.
  // Seed traj_server yaw history from actual aligned attitude
  // before t starts advancing.
  last_yaw_ = odom_yaw_;
  last_yaw_dot_ = 0.0;

  ROS_WARN(
      "[Traj server] TRIGGER YAW SYNC: "
      "actual_yaw=%.2f deg, odom_age=%.3f s",
      last_yaw_ * 57.29577951308232,
      odom_age
  );

  ros::Time t0 = msg->header.stamp;

  if (t0.isZero())
    t0 = now;

  // Restart the SAME loaded B-spline from t = 0.
  start_time_ = t0;
  execution_started_ = true;

  ROS_WARN(
      "[Traj server] EXECUTION TRIGGER: "
      "traj_id=%d reset to t=0, stamp=%.6f",
      traj_id_,
      t0.toSec()
  );
}


void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  //UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // Single-UAV real-flight execution:
  // start when traj_server actually receives the new B-spline.
  start_time_ = ros::Time::now();
  traj_id_ = msg->traj_id;

  // EXEC_PREVIEW_GOAL_GATE_V5
  //
  // Freeze:
  //   1) the first trajectory after node startup, or
  //   2) the next trajectory following a new MANUAL goal, or
  //   3) any additional trajectory received while still in preview.
  //
  // DO NOT freeze a normal in-flight EGO replan after execution
  // has already been released.
  const bool freeze_for_preview =
      (!receive_traj_) ||
      preview_request_pending_ ||
      (!execution_started_);

  if (preview_request_pending_)
    preview_request_pending_ = false;

  if (freeze_for_preview)
  {
    execution_started_ = false;
  }
  else
  {
    ROS_WARN(
        "[Traj server] IN-FLIGHT REPLAN: "
        "traj_id=%d continues execution without preview freeze.",
        traj_id_
    );
  }

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  ROS_WARN_STREAM("[Traj server]: BSP RECEIVED id="
                  << traj_id_
                  << " duration="
                  << traj_duration_
                  << " start_now="
                  << start_time_.toSec());

  receive_traj_ = true;

  if (!execution_started_)
  {
    ROS_WARN(
        "[Traj server] PREVIEW FREEZE: "
        "traj_id=%d held at t=0 until /traj_start_trigger.",
        traj_id_
    );
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = 0.5;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  // EXEC_PREVIEW_FREEZE_V3:
  // Before explicit execution trigger, keep publishing the
  // SAME B-spline at t=0.  This lets the interface inspect
  // first-gap and perform yaw alignment without allowing the
  // trajectory clock to advance.
  double t_cur = execution_started_
                     ? (time_now - start_time_).toSec()
                     : 0.0;

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
    // Keep publishing the final position for hover.
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

  // EXEC_PREVIEW_GOAL_GATE_V5
  // POINT's goal is remapped elsewhere; EGO retains the
  // absolute /move_base_simple/goal topic.
  ros::Subscriber preview_goal_sub =
      nh.subscribe(
          "/contest2/move_base_simple/goal",
          10,
          previewGoalCallback);

  ros::Subscriber odom_sub =
      nh.subscribe(
          "/contest2/mavros/local_position/odom",
          50,
          odomCallback);

  ros::Subscriber start_trigger_sub =
      nh.subscribe(
          "/contest2/traj_start_trigger",
          10,
          startTriggerCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/contest2/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}