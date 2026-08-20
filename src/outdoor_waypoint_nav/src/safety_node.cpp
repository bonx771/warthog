#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

ros::Publisher cmd_pub;
ros::Publisher replan_pub;
tf::TransformListener* tf_listener = nullptr;

geometry_msgs::Twist latest_cmd;
geometry_msgs::PoseStamped latest_goal;
geometry_msgs::PoseStamped previous_goal;
sensor_msgs::LaserScan latest_scan;
ros::Time last_cmd_time(0);
ros::Time last_scan_time(0);
ros::Time waypoint_align_start_time(0);
ros::Time obstacle_block_start_time(0);
ros::Time last_replan_request_time(0);

bool have_cmd = false;
bool have_scan = false;
bool have_goal = false;
bool have_previous_goal = false;
bool safety_enabled = true;
bool goal_slowdown_enabled = true;
bool waypoint_align_enabled = true;
bool waypoint_align_active = false;
bool waypoint_align_committed = false;
bool safety_scan_enabled = true;
bool safety_require_fresh_scan = true;
bool safety_command_timeout_stop_enabled = true;
bool obstacle_block_active = false;
bool safety_replan_enabled = true;

double stop_distance = 0.35;
double slowdown_distance = 0.80;
double sector_half_angle_deg = 20.0;
double command_timeout = 0.5;
double scan_timeout = 0.5;
double goal_slowdown_distance = 3.0;
double goal_slowdown_stop_distance = 0.6;
double goal_slowdown_min_forward_speed = 0.20;
double goal_slowdown_max_forward_speed = 1.4;
double waypoint_align_angle_threshold = 0.35;
double waypoint_align_release_angle = 0.12;
double waypoint_align_min_goal_distance = 1.20;
double waypoint_align_kp = 1.6;
double waypoint_align_min_angular_speed = 0.25;
double waypoint_align_max_angular_speed = 0.90;
double waypoint_align_timeout = 6.0;
double safety_replan_blocked_duration = 1.0;
double safety_replan_publish_cooldown = 2.0;

std::string input_cmd_topic = "/cmd_vel_intermediate";
std::string output_cmd_topic = "/cmd_vel";
std::string scan_topic = "/front/scan";
std::string goal_topic = "/move_base/current_goal";
std::string goal_base_frame = "base_link";
std::string safety_replan_topic = "/outdoor_waypoint_nav/replan_requested";

double normalizeAngle(double angle)
{
    while(angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while(angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool scanReadingIsUsable(const sensor_msgs::LaserScan& scan, float range)
{
    return std::isfinite(range) &&
           range >= scan.range_min &&
           range <= scan.range_max;
}

void cmdCB(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    latest_cmd = *cmd_msg;
    have_cmd = true;
    last_cmd_time = ros::Time::now();
}

void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    latest_scan = *scan_msg;
    have_scan = true;
    last_scan_time = ros::Time::now();
}

void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    bool new_goal = !have_previous_goal ||
                    previous_goal.header.frame_id != goal_msg->header.frame_id;
    if(have_previous_goal && !new_goal)
    {
        const double dx =
            goal_msg->pose.position.x - previous_goal.pose.position.x;
        const double dy =
            goal_msg->pose.position.y - previous_goal.pose.position.y;
        new_goal = std::sqrt((dx * dx) + (dy * dy)) > 0.05;
    }

    latest_goal = *goal_msg;
    have_goal = true;

    if(new_goal)
    {
        previous_goal = latest_goal;
        have_previous_goal = true;
        if(waypoint_align_enabled)
        {
            waypoint_align_active = true;
            waypoint_align_committed = false;
            waypoint_align_start_time = ros::Time::now();
            ROS_INFO(
                "safety_node: new waypoint goal received; checking heading "
                "before allowing forward motion.");
        }
    }
}

void publishStop()
{
    cmd_pub.publish(geometry_msgs::Twist());
}

bool scanIsFresh()
{
    if(!have_scan)
    {
        return false;
    }

    if(scan_timeout <= 0.0)
    {
        return true;
    }

    return (ros::Time::now() - last_scan_time).toSec() <= scan_timeout;
}

double frontMinimumRange()
{
    if(!have_scan)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double sector_half_angle_rad =
        std::fabs(sector_half_angle_deg) * M_PI / 180.0;
    double minimum_range = std::numeric_limits<double>::infinity();

    for(std::size_t i = 0; i < latest_scan.ranges.size(); ++i)
    {
        const float range = latest_scan.ranges[i];
        if(!scanReadingIsUsable(latest_scan, range))
        {
            continue;
        }

        const double angle = normalizeAngle(
            latest_scan.angle_min +
            static_cast<double>(i) * latest_scan.angle_increment);

        if(std::fabs(angle) <= sector_half_angle_rad)
        {
            minimum_range =
                std::min(minimum_range, static_cast<double>(range));
        }
    }

    return minimum_range;
}

double forwardSpeedScale(double obstacle_range)
{
    if(!std::isfinite(obstacle_range) ||
       obstacle_range >= slowdown_distance)
    {
        return 1.0;
    }

    if(obstacle_range <= stop_distance)
    {
        return 0.0;
    }

    const double available_distance = slowdown_distance - stop_distance;
    if(available_distance <= 1e-6)
    {
        return 0.0;
    }

    return std::max(
        0.0,
        std::min(
            1.0,
            (obstacle_range - stop_distance) / available_distance));
}

void resetObstacleBlockTracking()
{
    obstacle_block_active = false;
    obstacle_block_start_time = ros::Time(0);
}

void publishReplanRequest(double obstacle_range)
{
    if(!safety_replan_enabled || !have_goal)
    {
        return;
    }

    const ros::Time now = ros::Time::now();
    if(last_replan_request_time.toSec() > 0.0 &&
       safety_replan_publish_cooldown > 0.0 &&
       (now - last_replan_request_time).toSec() <
           safety_replan_publish_cooldown)
    {
        return;
    }

    std_msgs::Bool request_msg;
    request_msg.data = true;
    replan_pub.publish(request_msg);
    last_replan_request_time = now;

    ROS_WARN(
        "safety_node: obstacle blocked forward motion for %.2fs at %.2fm; "
        "requested move_base replan.",
        safety_replan_blocked_duration,
        obstacle_range);
}

void updateObstacleBlockReplan(double obstacle_range)
{
    if(!safety_replan_enabled)
    {
        return;
    }

    const ros::Time now = ros::Time::now();
    if(!obstacle_block_active)
    {
        obstacle_block_active = true;
        obstacle_block_start_time = now;
        return;
    }

    if(safety_replan_blocked_duration <= 0.0 ||
       (now - obstacle_block_start_time).toSec() >=
           safety_replan_blocked_duration)
    {
        publishReplanRequest(obstacle_range);
    }
}

bool tryGetGoalGeometry(double& distance, double& heading_error)
{
    if((!goal_slowdown_enabled && !waypoint_align_enabled) ||
       !have_goal ||
       tf_listener == nullptr ||
       latest_goal.header.frame_id.empty())
    {
        return false;
    }

    try
    {
        tf::StampedTransform robot_transform;
        tf_listener->lookupTransform(
            latest_goal.header.frame_id,
            goal_base_frame,
            ros::Time(0),
            robot_transform);

        const double dx =
            latest_goal.pose.position.x - robot_transform.getOrigin().x();
        const double dy =
            latest_goal.pose.position.y - robot_transform.getOrigin().y();
        distance = std::sqrt((dx * dx) + (dy * dy));

        const tf::Vector3 goal_in_goal_frame(
            latest_goal.pose.position.x,
            latest_goal.pose.position.y,
            latest_goal.pose.position.z);
        const tf::Vector3 goal_in_base_frame =
            robot_transform.inverse() * goal_in_goal_frame;
        heading_error = std::atan2(
            goal_in_base_frame.y(),
            goal_in_base_frame.x());
        return true;
    }
    catch(const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(
            2.0,
            "safety_node: could not measure current goal geometry: %s",
            ex.what());
        return false;
    }
}

void limitForwardSpeedNearGoal(
    geometry_msgs::Twist& cmd,
    bool have_goal_geometry,
    double goal_distance)
{
    if(!goal_slowdown_enabled ||
       !have_goal_geometry ||
       cmd.linear.x <= 0.0 ||
       goal_distance >= goal_slowdown_distance)
    {
        return;
    }

    const double available_distance =
        goal_slowdown_distance - goal_slowdown_stop_distance;
    const double ratio =
        available_distance > 1e-6 ?
        std::max(
            0.0,
            std::min(
                1.0,
                (goal_distance - goal_slowdown_stop_distance) /
                available_distance)) :
        0.0;
    const double min_speed =
        std::max(0.0, std::fabs(goal_slowdown_min_forward_speed));
    const double max_speed =
        std::max(min_speed, std::fabs(goal_slowdown_max_forward_speed));
    const double allowed_speed =
        min_speed + ratio * (max_speed - min_speed);

    if(cmd.linear.x > allowed_speed)
    {
        cmd.linear.x = allowed_speed;
        ROS_INFO_THROTTLE(
            0.8,
            "safety_node: goal %.2fm away; limiting forward speed to %.2fm/s.",
            goal_distance,
            allowed_speed);
    }
}

double signedClampedTurnRate(double heading_error)
{
    const double max_turn =
        std::max(0.01, std::fabs(waypoint_align_max_angular_speed));
    const double min_turn =
        std::min(max_turn, std::max(0.0, std::fabs(waypoint_align_min_angular_speed)));
    double turn_rate =
        std::max(-max_turn, std::min(max_turn, waypoint_align_kp * heading_error));

    if(std::fabs(turn_rate) < min_turn &&
       std::fabs(heading_error) > waypoint_align_release_angle)
    {
        turn_rate = (heading_error > 0.0 ? min_turn : -min_turn);
    }

    return turn_rate;
}

void updateWaypointAlignmentState(
    bool have_goal_geometry,
    double goal_distance,
    double heading_error)
{
    if(!waypoint_align_enabled || !waypoint_align_active)
    {
        return;
    }

    if(!have_goal_geometry ||
       goal_distance <= waypoint_align_min_goal_distance)
    {
        waypoint_align_active = false;
        waypoint_align_committed = false;
        return;
    }

    const double abs_error = std::fabs(heading_error);
    if(!waypoint_align_committed && abs_error < waypoint_align_angle_threshold)
    {
        waypoint_align_active = false;
        return;
    }

    waypoint_align_committed = true;
    if(abs_error <= waypoint_align_release_angle)
    {
        waypoint_align_active = false;
        waypoint_align_committed = false;
        ROS_INFO(
            "safety_node: waypoint heading aligned (error %.1f deg); "
            "forward motion released.",
            heading_error * 180.0 / M_PI);
        return;
    }

    if(waypoint_align_timeout > 1e-6 &&
       waypoint_align_start_time.toSec() > 0.0 &&
       (ros::Time::now() - waypoint_align_start_time).toSec() >
           waypoint_align_timeout)
    {
        waypoint_align_active = false;
        waypoint_align_committed = false;
        ROS_WARN(
            "safety_node: waypoint heading alignment timed out with %.1f deg "
            "remaining; releasing forward motion.",
            heading_error * 180.0 / M_PI);
    }
}

bool forceRotateTowardWaypoint(
    geometry_msgs::Twist& cmd,
    bool have_goal_geometry,
    double goal_distance,
    double heading_error)
{
    if(!waypoint_align_enabled ||
       !waypoint_align_active ||
       !have_goal_geometry ||
       goal_distance <= waypoint_align_min_goal_distance)
    {
        return false;
    }

    cmd = geometry_msgs::Twist();
    cmd.angular.z = signedClampedTurnRate(heading_error);

    ROS_INFO_THROTTLE(
        0.5,
        "safety_node: holding position to face waypoint. dist=%.2fm err=%.1fdeg cmd_w=%.2f",
        goal_distance,
        heading_error * 180.0 / M_PI,
        cmd.angular.z);
    return true;
}

void controlTimerCB(const ros::TimerEvent&)
{
    const bool command_is_fresh =
        have_cmd &&
        (ros::Time::now() - last_cmd_time).toSec() <= command_timeout;

    if(!command_is_fresh)
    {
        if(safety_enabled && safety_command_timeout_stop_enabled)
        {
            publishStop();
        }
        resetObstacleBlockTracking();
        return;
    }

    if(!safety_enabled)
    {
        cmd_pub.publish(latest_cmd);
        return;
    }

    double goal_distance = std::numeric_limits<double>::infinity();
    double goal_heading_error = 0.0;
    const bool have_goal_geometry =
        tryGetGoalGeometry(goal_distance, goal_heading_error);
    updateWaypointAlignmentState(
        have_goal_geometry,
        goal_distance,
        goal_heading_error);

    geometry_msgs::Twist output = latest_cmd;

    const bool fresh_scan = safety_scan_enabled && scanIsFresh();
    if(safety_scan_enabled && safety_require_fresh_scan && !fresh_scan)
    {
        publishStop();
        resetObstacleBlockTracking();
        ROS_WARN_THROTTLE(
            1.0,
            "safety_node: no fresh scan on %s for %.2fs; holding stop.",
            scan_topic.c_str(),
            scan_timeout);
        return;
    }

    if(forceRotateTowardWaypoint(
           output,
           have_goal_geometry,
           goal_distance,
           goal_heading_error))
    {
        cmd_pub.publish(output);
        return;
    }

    if(safety_scan_enabled && fresh_scan && output.linear.x > 0.0)
    {
        const double obstacle_range = frontMinimumRange();
        const double speed_scale = forwardSpeedScale(obstacle_range);
        output.linear.x *= speed_scale;

        if(speed_scale <= 0.0)
        {
            ROS_WARN_THROTTLE(
                1.0,
                "safety_node: obstacle at %.2fm; blocking forward motion "
                "while preserving move_base steering.",
                obstacle_range);
            updateObstacleBlockReplan(obstacle_range);
        }
        else if(speed_scale < 1.0)
        {
            resetObstacleBlockTracking();
            ROS_INFO_THROTTLE(
                1.0,
                "safety_node: obstacle at %.2fm; forward speed scaled to %.0f%%.",
                obstacle_range,
                speed_scale * 100.0);
        }
        else
        {
            resetObstacleBlockTracking();
        }
    }
    else
    {
        resetObstacleBlockTracking();
    }

    limitForwardSpeedNearGoal(output, have_goal_geometry, goal_distance);

    cmd_pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "safety_node");
    ros::NodeHandle nh;

    ros::param::param<bool>(
        "/outdoor_waypoint_nav/safety_enabled",
        safety_enabled,
        true);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_stop_distance",
        stop_distance,
        0.35);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_slowdown_distance",
        slowdown_distance,
        0.80);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_sector_half_angle_deg",
        sector_half_angle_deg,
        20.0);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_command_timeout",
        command_timeout,
        0.5);
    ros::param::param<bool>(
        "/outdoor_waypoint_nav/safety_command_timeout_stop_enabled",
        safety_command_timeout_stop_enabled,
        true);
    ros::param::param<bool>(
        "/outdoor_waypoint_nav/safety_scan_enabled",
        safety_scan_enabled,
        true);
    ros::param::param<bool>(
        "/outdoor_waypoint_nav/safety_require_fresh_scan",
        safety_require_fresh_scan,
        true);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_scan_timeout",
        scan_timeout,
        0.5);
    ros::param::param<bool>(
        "/outdoor_waypoint_nav/goal_slowdown_enabled",
        goal_slowdown_enabled,
        true);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/goal_slowdown_distance",
        goal_slowdown_distance,
        3.0);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/goal_slowdown_stop_distance",
        goal_slowdown_stop_distance,
        0.6);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/goal_slowdown_min_forward_speed",
        goal_slowdown_min_forward_speed,
        0.20);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/goal_slowdown_max_forward_speed",
        goal_slowdown_max_forward_speed,
        1.4);
    ros::param::param<bool>(
        "/outdoor_waypoint_nav/waypoint_align_enabled",
        waypoint_align_enabled,
        true);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_angle_threshold",
        waypoint_align_angle_threshold,
        0.35);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_release_angle",
        waypoint_align_release_angle,
        0.12);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_min_goal_distance",
        waypoint_align_min_goal_distance,
        1.20);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_kp",
        waypoint_align_kp,
        1.6);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_min_angular_speed",
        waypoint_align_min_angular_speed,
        0.25);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_max_angular_speed",
        waypoint_align_max_angular_speed,
        0.90);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/waypoint_align_timeout",
        waypoint_align_timeout,
        6.0);
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_input_cmd_topic",
        input_cmd_topic,
        "/cmd_vel_intermediate");
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_output_cmd_topic",
        output_cmd_topic,
        "/cmd_vel");
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_scan_topic",
        scan_topic,
        "/front/scan");
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/goal_slowdown_topic",
        goal_topic,
        "/move_base/current_goal");
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/goal_base_frame",
        goal_base_frame,
        "base_link");
    ros::param::param<bool>(
        "/outdoor_waypoint_nav/safety_replan_enabled",
        safety_replan_enabled,
        true);
    ros::param::param<std::string>(
        "/outdoor_waypoint_nav/safety_replan_topic",
        safety_replan_topic,
        "/outdoor_waypoint_nav/replan_requested");
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_replan_blocked_duration",
        safety_replan_blocked_duration,
        1.0);
    ros::param::param<double>(
        "/outdoor_waypoint_nav/safety_replan_publish_cooldown",
        safety_replan_publish_cooldown,
        2.0);

    if(slowdown_distance <= stop_distance)
    {
        ROS_WARN(
            "safety_node: slowdown distance %.2fm must be larger than stop "
            "distance %.2fm. Using %.2fm.",
            slowdown_distance,
            stop_distance,
            stop_distance + 0.10);
        slowdown_distance = stop_distance + 0.10;
    }
    if(goal_slowdown_distance <= goal_slowdown_stop_distance)
    {
        ROS_WARN(
            "safety_node: goal slowdown distance %.2fm must be larger than "
            "goal stop distance %.2fm. Using %.2fm.",
            goal_slowdown_distance,
            goal_slowdown_stop_distance,
            goal_slowdown_stop_distance + 0.10);
        goal_slowdown_distance = goal_slowdown_stop_distance + 0.10;
    }
    if(waypoint_align_angle_threshold < waypoint_align_release_angle)
    {
        ROS_WARN(
            "safety_node: waypoint align threshold %.2frad is smaller than "
            "release angle %.2frad. Using %.2frad.",
            waypoint_align_angle_threshold,
            waypoint_align_release_angle,
            waypoint_align_release_angle + 0.05);
        waypoint_align_angle_threshold = waypoint_align_release_angle + 0.05;
    }
    if(scan_timeout < 0.0)
    {
        ROS_WARN(
            "safety_node: scan timeout %.2fs is invalid. Disabling scan "
            "timeout.",
            scan_timeout);
        scan_timeout = 0.0;
    }
    if(!safety_scan_enabled && safety_require_fresh_scan)
    {
        ROS_WARN(
            "safety_node: scan safety is disabled, ignoring fresh scan "
            "requirement.");
    }
    if(safety_replan_blocked_duration < 0.0)
    {
        ROS_WARN(
            "safety_node: replan blocked duration %.2fs is invalid. Using 0.",
            safety_replan_blocked_duration);
        safety_replan_blocked_duration = 0.0;
    }
    if(safety_replan_publish_cooldown < 0.0)
    {
        ROS_WARN(
            "safety_node: replan publish cooldown %.2fs is invalid. Using 0.",
            safety_replan_publish_cooldown);
        safety_replan_publish_cooldown = 0.0;
    }

    cmd_pub = nh.advertise<geometry_msgs::Twist>(output_cmd_topic, 10);
    replan_pub = nh.advertise<std_msgs::Bool>(safety_replan_topic, 10);
    tf_listener = new tf::TransformListener(ros::Duration(10.0));

    ros::Subscriber cmd_sub = nh.subscribe(input_cmd_topic, 10, cmdCB);
    ros::Subscriber scan_sub;
    if(safety_scan_enabled)
    {
        scan_sub = nh.subscribe(scan_topic, 10, scanCB);
    }
    ros::Subscriber goal_sub = nh.subscribe(goal_topic, 10, goalCB);
    ros::Timer control_timer =
        nh.createTimer(ros::Duration(0.05), controlTimerCB);

    ROS_INFO(
        "safety_node: active. input=%s output=%s safety=%s timeout_stop=%s "
        "scan_safety=%s scan=%s slowdown=%.2fm stop=%.2fm "
        "sector=+/-%.1fdeg scan_timeout=%.2fs goal_slowdown=%s "
        "waypoint_align=%s replan=%s",
        input_cmd_topic.c_str(),
        output_cmd_topic.c_str(),
        safety_enabled ? "on" : "off",
        safety_command_timeout_stop_enabled ? "on" : "off",
        safety_scan_enabled ? "on" : "off",
        scan_topic.c_str(),
        slowdown_distance,
        stop_distance,
        sector_half_angle_deg,
        (safety_scan_enabled && safety_require_fresh_scan) ? scan_timeout : 0.0,
        goal_slowdown_enabled ? "on" : "off",
        waypoint_align_enabled ? "on" : "off",
        safety_replan_enabled ? "on" : "off");

    ros::spin();
    delete tf_listener;
    tf_listener = nullptr;
    return 0;
}
