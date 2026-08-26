#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <math.h>




// initialize variables

typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction>
MoveBaseClient; //create a type definition for a client called MoveBaseClient

std::vector <std::pair<double, double>> waypointVect;
std::vector<std::pair < double, double> > ::iterator iter; //init. iterator
geometry_msgs::PointStamped UTM_point, map_point, UTM_next, map_next;
int count = 0, waypointCount = 0, wait_count = 0;
double numWaypoints = 0;
double latiGoal, longiGoal, latiNext, longiNext;
std::string utm_zone;
std::string path_local, path_abs, goal_frame, waypoint_marker_frame, waypoint_marker_topic;
double waypoint_marker_scale = 0.7;
double waypoint_marker_activation_radius = 0.675;
double max_waypoint_goal_yaw_delta = 0.60;
double waypoint_heading_preview_radius = 1.0;
double waypoint_advance_radius = 0.0;
bool oscillation_recovery_enabled = true;
int oscillation_recovery_max_attempts = 2;
double oscillation_recovery_reverse_distance = 1.0;
double oscillation_recovery_turn_angle_deg = 90.0;
double oscillation_recovery_forward_distance = 1.5;
double oscillation_recovery_linear_speed = 0.30;
double oscillation_recovery_angular_speed = 0.45;
sensor_msgs::LaserScan latest_recovery_scan;
bool have_recovery_scan = false;
bool safety_replan_enabled = true;
bool safety_replan_requested = false;
std::string safety_replan_topic = "/outdoor_waypoint_nav/replan_requested";
double safety_replan_cooldown = 0.0;
ros::Time last_safety_replan_time(0);
bool auto_replan_current_goal_enabled = false;
double auto_replan_current_goal_interval = 1.5;
ros::Time last_auto_replan_time(0);
std::string preferred_avoidance_turn_topic = "/outdoor_waypoint_nav/preferred_avoidance_turn_direction";
double preferred_avoidance_turn_deadband_deg = 8.0;
double preferred_avoidance_turn_sign = 1.0;
std::string waypoint_cmd_topic = "/cmd_vel";
bool waypoint_pre_align_enabled = false;
double waypoint_pre_align_angle_threshold = 0.35;
double waypoint_pre_align_min_goal_distance = 0.80;
bool return_to_start_enabled = true;
bool return_heading_pid_enabled = true;
double return_heading_pid_kp = 1.4;
double return_heading_pid_ki = 0.02;
double return_heading_pid_kd = 0.08;
double return_heading_pid_tolerance_deg = 5.0;
double return_heading_pid_max_angular_speed = 0.8;
double return_heading_pid_min_angular_speed = 0.12;
double return_heading_pid_integral_limit = 1.0;
double return_heading_pid_settle_time = 0.4;
double return_heading_pid_timeout = 12.0;
bool direct_waypoint_drive_enabled = false;
double direct_waypoint_goal_tolerance = 0.10;
double direct_waypoint_final_goal_tolerance = 0.10;
double direct_waypoint_cruise_speed = 0.45;
double direct_waypoint_min_speed = 0.12;
double direct_waypoint_slow_radius = 2.0;
double direct_waypoint_angular_kp = 1.25;
double direct_waypoint_max_angular_speed = 0.60;
double direct_waypoint_min_turn_speed = 0.16;
double direct_waypoint_in_place_angle_threshold = 1.55;
double direct_waypoint_progress_timeout = 20.0;
double direct_waypoint_progress_epsilon = 0.05;
double direct_waypoint_control_frequency = 20.0;
double direct_waypoint_near_goal_heading_radius = 0.0;
double direct_waypoint_near_goal_max_speed = 0.08;
double direct_waypoint_near_goal_max_angular_speed = 0.15;


int countWaypointsInFile(std::string path_local)
{
    path_abs = ros::package::getPath("outdoor_waypoint_nav") + path_local;
    std::ifstream fileCount(path_abs.c_str());
    if(fileCount.is_open())
    {
        double lati = 0;
        while(!fileCount.eof())
        {
            fileCount >> lati;
            ++count;
        }
        count = count - 1;
        numWaypoints = count / 2;
        ROS_INFO("%.0f GPS waypoints were read", numWaypoints);
        fileCount.close();
    }
    else
    {
        std::cout << "Unable to open waypoint file" << std::endl;
        ROS_ERROR("Unable to open waypoint file");
    }
    return numWaypoints;
}

std::vector <std::pair<double, double>> getWaypoints(std::string path_local)
{
    double lati = 0, longi = 0;

    path_abs = ros::package::getPath("outdoor_waypoint_nav") + path_local;
    std::ifstream fileRead(path_abs.c_str());
    for(int i = 0; i < numWaypoints; i++)
    {
        fileRead >> lati;
        fileRead >> longi;
        waypointVect.push_back(std::make_pair(lati, longi));
    }
    fileRead.close();

    //Outputting vector
    ROS_INFO("The following GPS Waypoints have been set:");
    for(std::vector < std::pair < double, double >> ::iterator iterDisp = waypointVect.begin(); iterDisp != waypointVect.end();
    iterDisp++)
    {
        ROS_INFO("%.9g %.9g", iterDisp->first, iterDisp->second);
    }
    return waypointVect;
}

geometry_msgs::PointStamped latLongtoUTM(double lati_input, double longi_input)
{
    double utm_x = 0, utm_y = 0;
    geometry_msgs::PointStamped UTM_point_output;

    //convert lat/long to utm
    RobotLocalization::NavsatConversions::LLtoUTM(lati_input, longi_input, utm_y, utm_x, utm_zone);

    //Construct UTM_point and map_point geometry messages
    UTM_point_output.header.frame_id = "utm";
    UTM_point_output.header.stamp = ros::Time(0);
    UTM_point_output.point.x = utm_x;
    UTM_point_output.point.y = utm_y;
    UTM_point_output.point.z = 0;

    return UTM_point_output;
}

geometry_msgs::PointStamped UTMtoMapPoint(geometry_msgs::PointStamped UTM_input)
{
    geometry_msgs::PointStamped map_point_output;
    bool notDone = true;
    tf::TransformListener listener; //create transformlistener object called listener
    ros::Time time_now = ros::Time::now();
    while(notDone)
    {
        try
        {
            UTM_point.header.stamp = ros::Time::now();
            listener.waitForTransform(goal_frame, "utm", time_now, ros::Duration(3.0));
            listener.transformPoint(goal_frame, UTM_input, map_point_output);
            notDone = false;
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.01).sleep();
            //return;
        }
    }
    return map_point_output;
}

double normalizeAngle(double angle)
{
    while(angle > M_PI)
    {
        angle -= (2.0 * M_PI);
    }

    while(angle < -M_PI)
    {
        angle += (2.0 * M_PI);
    }

    return angle;
}

double clampDouble(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(max_value, value));
}

bool scanReadingIsUsable(const sensor_msgs::LaserScan& scan, float range)
{
    return std::isfinite(range) && range >= scan.range_min && range <= scan.range_max;
}

void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    latest_recovery_scan = *scan_msg;
    have_recovery_scan = true;
}

void safetyReplanCB(const std_msgs::Bool::ConstPtr& request_msg)
{
    if(request_msg->data)
    {
        safety_replan_requested = true;
    }
}

bool consumeSafetyReplanRequest()
{
    if(!safety_replan_enabled || !safety_replan_requested)
    {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if(safety_replan_cooldown > 0.0 &&
       last_safety_replan_time.toSec() > 0.0 &&
       (now - last_safety_replan_time).toSec() < safety_replan_cooldown)
    {
        safety_replan_requested = false;
        return false;
    }

    safety_replan_requested = false;
    last_safety_replan_time = now;
    return true;
}

bool shouldAutoReplanCurrentGoal()
{
    if(!auto_replan_current_goal_enabled || auto_replan_current_goal_interval <= 0.0)
    {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if(last_auto_replan_time.toSec() <= 0.0)
    {
        last_auto_replan_time = now;
        return false;
    }

    if((now - last_auto_replan_time).toSec() < auto_replan_current_goal_interval)
    {
        return false;
    }

    last_auto_replan_time = now;
    return true;
}

bool chooseRecoveryTurnDirection(double& turn_direction)
{
    if(!have_recovery_scan)
    {
        turn_direction = 1.0;
        return false;
    }

    double left_clearance = 0.0;
    double right_clearance = 0.0;
    int left_count = 0;
    int right_count = 0;

    for(std::size_t i = 0; i < latest_recovery_scan.ranges.size(); ++i)
    {
        const float range = latest_recovery_scan.ranges[i];
        if(!scanReadingIsUsable(latest_recovery_scan, range))
        {
            continue;
        }

        const double angle =
            normalizeAngle(latest_recovery_scan.angle_min + (static_cast<double>(i) * latest_recovery_scan.angle_increment));
        if(std::fabs(angle) > (M_PI / 2.0))
        {
            continue;
        }

        if(angle >= 0.0)
        {
            left_clearance += range;
            left_count++;
        }
        else
        {
            right_clearance += range;
            right_count++;
        }
    }

    const double left_metric = (left_count > 0) ? (left_clearance / left_count) : 0.0;
    const double right_metric = (right_count > 0) ? (right_clearance / right_count) : 0.0;
    turn_direction = (left_metric >= right_metric) ? 1.0 : -1.0;
    return (left_count > 0 || right_count > 0);
}

bool tryGetRobotYawInFrame(
    tf::TransformListener& listener,
    const std::string& target_frame,
    double& yaw)
{
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform(target_frame, "base_link", ros::Time(0), transform);
        yaw = tf::getYaw(transform.getRotation());
        return true;
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(2.0, "Unable to get robot yaw for waypoint goal orientation: %s", ex.what());
        return false;
    }
}

move_base_msgs::MoveBaseGoal buildGoal(
    bool have_prev_point,
    geometry_msgs::PointStamped map_prev,
    geometry_msgs::PointStamped map_point,
    bool have_next_point,
    geometry_msgs::PointStamped map_next,
    bool last_point,
    bool use_preview_heading,
    bool have_current_yaw,
    double current_yaw)
{
    move_base_msgs::MoveBaseGoal goal;

    //Specify what frame we want the goal to be published in
    goal.target_pose.header.frame_id = goal_frame;
    goal.target_pose.header.stamp = ros::Time::now();

    // Specify x and y goal
    goal.target_pose.pose.position.x = map_point.point.x; //specify x goal
    goal.target_pose.pose.position.y = map_point.point.y; //specify y goal

    // For intermediate waypoints, keep the initial goal aligned with the current segment.
    // Only when the robot is near the waypoint do we resend the goal with a preview heading
    // toward the outgoing segment.
    if(last_point == false)
    {
        tf::Matrix3x3 rot_euler;
        tf::Quaternion rot_quat;
        const double x_curr = map_point.point.x;
        const double y_curr = map_point.point.y;
        const double x_next = map_next.point.x;
        const double y_next = map_next.point.y;
        const double delta_x_out = x_next - x_curr;
        const double delta_y_out = y_next - y_curr;
        double yaw_curr = atan2(delta_y_out, delta_x_out);

        if(have_prev_point && !use_preview_heading)
        {
            const double x_prev = map_prev.point.x;
            const double y_prev = map_prev.point.y;
            yaw_curr = atan2(y_curr - y_prev, x_curr - x_prev);
        }
        else if(have_prev_point && use_preview_heading)
        {
            const double x_prev = map_prev.point.x;
            const double y_prev = map_prev.point.y;
            const double delta_x_in = x_curr - x_prev;
            const double delta_y_in = y_curr - y_prev;
            const double incoming_norm = std::sqrt((delta_x_in * delta_x_in) + (delta_y_in * delta_y_in));
            const double outgoing_norm = std::sqrt((delta_x_out * delta_x_out) + (delta_y_out * delta_y_out));

            if(incoming_norm > 1e-6 && outgoing_norm > 1e-6)
            {
                const double preview_x = (delta_x_in / incoming_norm) + (delta_x_out / outgoing_norm);
                const double preview_y = (delta_y_in / incoming_norm) + (delta_y_out / outgoing_norm);

                if(std::sqrt((preview_x * preview_x) + (preview_y * preview_y)) > 1e-6)
                {
                    yaw_curr = atan2(preview_y, preview_x);
                }
            }
        }
        else if(!have_prev_point && !use_preview_heading && have_current_yaw)
        {
            yaw_curr = current_yaw;
        }

        if(have_current_yaw)
        {
            const double yaw_error = normalizeAngle(yaw_curr - current_yaw);
            const double clamped_yaw_error = std::max(-max_waypoint_goal_yaw_delta, std::min(max_waypoint_goal_yaw_delta, yaw_error));
            yaw_curr = current_yaw + clamped_yaw_error;
        }

        // Specify quaternions
        rot_euler.setEulerYPR(yaw_curr, 0.0, 0.0);
        rot_euler.getRotation(rot_quat);

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }
    else
    {
        tf::Quaternion rot_quat;
        if(have_current_yaw)
        {
            rot_quat.setRPY(0.0, 0.0, current_yaw);
        }
        else
        {
            rot_quat.setRPY(0.0, 0.0, 0.0);
        }

        goal.target_pose.pose.orientation.x = rot_quat.getX();
        goal.target_pose.pose.orientation.y = rot_quat.getY();
        goal.target_pose.pose.orientation.z = rot_quat.getZ();
        goal.target_pose.pose.orientation.w = rot_quat.getW();
    }

    return goal;
}

void publishReachedWaypointMarker(
    ros::Publisher& marker_pub,
    const geometry_msgs::PointStamped& marker_point,
    int waypoint_index)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = waypoint_marker_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "collected_waypoints";
    marker.id = waypoint_index;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = marker_point.point.x;
    marker.pose.position.y = marker_point.point.y;
    marker.pose.position.z = marker_point.point.z + 0.2;
    marker.scale.x = waypoint_marker_scale;
    marker.scale.y = waypoint_marker_scale;
    marker.scale.z = waypoint_marker_scale;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.0);
    marker_pub.publish(marker);
}

void publishReachedWaypointIndex(
    ros::Publisher& waypoint_index_pub,
    int waypoint_index)
{
    std_msgs::Int32 waypoint_index_msg;
    waypoint_index_msg.data = waypoint_index;
    waypoint_index_pub.publish(waypoint_index_msg);
}

bool tryGetRobotPointInFrame(
    tf::TransformListener& listener,
    const std::string& target_frame,
    geometry_msgs::PointStamped& robot_point)
{
    geometry_msgs::PointStamped base_point;
    base_point.header.frame_id = "base_link";
    base_point.header.stamp = ros::Time(0);
    base_point.point.x = 0.0;
    base_point.point.y = 0.0;
    base_point.point.z = 0.0;

    try
    {
        listener.transformPoint(target_frame, base_point, robot_point);
        return true;
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(2.0, "Unable to update waypoint proximity marker: %s", ex.what());
        return false;
    }
}

struct RobotPose2D
{
    double x;
    double y;
    double yaw;
};

bool tryGetRobotPoseInFrame(
    tf::TransformListener& listener,
    const std::string& target_frame,
    RobotPose2D& pose)
{
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform(target_frame, "base_link", ros::Time(0), transform);
        pose.x = transform.getOrigin().x();
        pose.y = transform.getOrigin().y();
        pose.yaw = tf::getYaw(transform.getRotation());
        return true;
    }
    catch (tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(2.0, "Unable to get robot pose in %s for oscillation recovery: %s", target_frame.c_str(), ex.what());
        return false;
    }
}

void publishPreferredAvoidanceTurn(
    ros::Publisher& preferred_turn_pub,
    tf::TransformListener& listener,
    const geometry_msgs::PointStamped& current_waypoint,
    const geometry_msgs::PointStamped& next_waypoint,
    bool final_point,
    bool)
{
    RobotPose2D robot_pose;
    std_msgs::Float64 preferred_turn_msg;
    preferred_turn_msg.data = 0.0;

    if(!tryGetRobotPoseInFrame(listener, goal_frame, robot_pose))
    {
        preferred_turn_pub.publish(preferred_turn_msg);
        return;
    }

    const bool should_lookahead = !final_point;

    const geometry_msgs::PointStamped& target_waypoint =
        should_lookahead ? next_waypoint : current_waypoint;
    const double target_dx = target_waypoint.point.x - robot_pose.x;
    const double target_dy = target_waypoint.point.y - robot_pose.y;
    if(std::sqrt((target_dx * target_dx) + (target_dy * target_dy)) < 1e-3)
    {
        preferred_turn_pub.publish(preferred_turn_msg);
        return;
    }

    const double desired_yaw = std::atan2(target_dy, target_dx);
    const double yaw_error = normalizeAngle(desired_yaw - robot_pose.yaw);
    const double deadband_rad = preferred_avoidance_turn_deadband_deg * M_PI / 180.0;
    if(std::fabs(yaw_error) > deadband_rad)
    {
        preferred_turn_msg.data =
            (yaw_error > 0.0 ? 1.0 : -1.0) *
            (preferred_avoidance_turn_sign >= 0.0 ? 1.0 : -1.0);
    }

    preferred_turn_pub.publish(preferred_turn_msg);
}

void publishStop(ros::Publisher& cmd_pub)
{
    geometry_msgs::Twist stop_cmd;
    ros::Rate rate(20.0);
    for(int i = 0; i < 5 && ros::ok(); ++i)
    {
        cmd_pub.publish(stop_cmd);
        rate.sleep();
    }
}

bool waitForRobotYawInFrame(
    tf::TransformListener& listener,
    const std::string& target_frame,
    double timeout,
    double& yaw)
{
    ros::Rate rate(20.0);
    const ros::Time start_time = ros::Time::now();

    while(ros::ok() && ((ros::Time::now() - start_time).toSec() < timeout))
    {
        ros::spinOnce();
        if(tryGetRobotYawInFrame(listener, target_frame, yaw))
        {
            return true;
        }
        rate.sleep();
    }

    return tryGetRobotYawInFrame(listener, target_frame, yaw);
}

bool alignYawToTargetPID(
    tf::TransformListener& listener,
    ros::Publisher& cmd_pub,
    const std::string& target_frame,
    double target_yaw)
{
    const double tolerance_rad = return_heading_pid_tolerance_deg * M_PI / 180.0;
    const double max_angular_speed = std::max(0.01, std::fabs(return_heading_pid_max_angular_speed));
    const double min_angular_speed =
        std::min(max_angular_speed, std::max(0.0, std::fabs(return_heading_pid_min_angular_speed)));
    const double integral_limit = std::max(0.0, return_heading_pid_integral_limit);
    const double settle_time = std::max(0.0, return_heading_pid_settle_time);
    const double timeout = std::max(1.0, return_heading_pid_timeout);

    double previous_error = 0.0;
    double error_integral = 0.0;
    bool have_previous_error = false;
    ros::Time previous_time = ros::Time::now();
    ros::Time settled_since(0);
    const ros::Time start_time = ros::Time::now();

    ROS_INFO(
        "Return heading PID started. Target yaw: %.1f deg, tolerance: %.1f deg.",
        target_yaw * 180.0 / M_PI,
        return_heading_pid_tolerance_deg);

    ros::Rate rate(30.0);
    while(ros::ok() && ((ros::Time::now() - start_time).toSec() < timeout))
    {
        ros::spinOnce();

        RobotPose2D current_pose;
        if(!tryGetRobotPoseInFrame(listener, target_frame, current_pose))
        {
            publishStop(cmd_pub);
            rate.sleep();
            continue;
        }

        const ros::Time now = ros::Time::now();
        const double dt = std::max(1e-3, (now - previous_time).toSec());
        const double error = normalizeAngle(target_yaw - current_pose.yaw);
        const double abs_error = std::fabs(error);

        if(abs_error <= tolerance_rad)
        {
            publishStop(cmd_pub);
            if(settled_since.toSec() <= 0.0)
            {
                settled_since = now;
            }

            if((now - settled_since).toSec() >= settle_time)
            {
                ROS_INFO(
                    "Return heading PID finished. Final yaw error: %.2f deg.",
                    error * 180.0 / M_PI);
                return true;
            }

            previous_time = now;
            previous_error = error;
            have_previous_error = true;
            rate.sleep();
            continue;
        }

        settled_since = ros::Time(0);

        error_integral += error * dt;
        if(integral_limit > 0.0)
        {
            error_integral = std::max(-integral_limit, std::min(integral_limit, error_integral));
        }

        const double error_derivative = have_previous_error ? (normalizeAngle(error - previous_error) / dt) : 0.0;
        double angular_cmd =
            (return_heading_pid_kp * error) +
            (return_heading_pid_ki * error_integral) +
            (return_heading_pid_kd * error_derivative);

        angular_cmd = std::max(-max_angular_speed, std::min(max_angular_speed, angular_cmd));
        if(std::fabs(angular_cmd) < min_angular_speed)
        {
            angular_cmd = (error >= 0.0 ? 1.0 : -1.0) * min_angular_speed;
        }

        geometry_msgs::Twist cmd;
        cmd.angular.z = angular_cmd;
        cmd_pub.publish(cmd);

        previous_time = now;
        previous_error = error;
        have_previous_error = true;
        rate.sleep();
    }

    publishStop(cmd_pub);

    RobotPose2D final_pose;
    if(tryGetRobotPoseInFrame(listener, target_frame, final_pose))
    {
        ROS_WARN(
            "Return heading PID timed out after %.1fs. Final yaw error: %.2f deg.",
            timeout,
            normalizeAngle(target_yaw - final_pose.yaw) * 180.0 / M_PI);
    }
    else
    {
        ROS_WARN("Return heading PID timed out after %.1fs and final yaw could not be read.", timeout);
    }

    return false;
}

void alignTowardWaypointIfNeeded(
    tf::TransformListener& listener,
    ros::Publisher& cmd_pub,
    const geometry_msgs::PointStamped& waypoint,
    int waypoint_index,
    int total_waypoints)
{
    RobotPose2D robot_pose;
    if(!tryGetRobotPoseInFrame(listener, goal_frame, robot_pose))
    {
        return;
    }

    const double dx = waypoint.point.x - robot_pose.x;
    const double dy = waypoint.point.y - robot_pose.y;
    const double distance = std::sqrt((dx * dx) + (dy * dy));
    const double desired_yaw = std::atan2(dy, dx);
    const double yaw_error = normalizeAngle(desired_yaw - robot_pose.yaw);

    ROS_INFO(
        "Waypoint %d/%d robot-to-goal: dist=%.2fm bearing=%.1fdeg yaw=%.1fdeg err=%.1fdeg.",
        waypoint_index,
        total_waypoints,
        distance,
        desired_yaw * 180.0 / M_PI,
        robot_pose.yaw * 180.0 / M_PI,
        yaw_error * 180.0 / M_PI);

    if(!waypoint_pre_align_enabled ||
       distance <= waypoint_pre_align_min_goal_distance ||
       std::fabs(yaw_error) <= waypoint_pre_align_angle_threshold)
    {
        return;
    }

    ROS_INFO(
        "Pre-aligning toward waypoint %d/%d before sending move_base goal.",
        waypoint_index,
        total_waypoints);
    alignYawToTargetPID(listener, cmd_pub, goal_frame, desired_yaw);
}

bool driveDirectToWaypoint(
    tf::TransformListener& listener,
    ros::Publisher& cmd_pub,
    ros::Publisher& marker_pub,
    const geometry_msgs::PointStamped& waypoint,
    int waypoint_index,
    int total_waypoints)
{
    const bool final_waypoint = waypoint_index >= total_waypoints;
    const double configured_tolerance =
        (final_waypoint && direct_waypoint_final_goal_tolerance > 0.0) ?
            direct_waypoint_final_goal_tolerance :
            direct_waypoint_goal_tolerance;
    const double fallback_tolerance =
        (waypoint_advance_radius > 0.0) ? waypoint_advance_radius : 1.0;
    const double goal_tolerance = std::max(
        0.05,
        configured_tolerance > 0.0 ? configured_tolerance : fallback_tolerance);
    const double cruise_speed = std::max(0.0, direct_waypoint_cruise_speed);
    const double min_speed = std::min(cruise_speed, std::max(0.0, direct_waypoint_min_speed));
    const double slow_radius = std::max(goal_tolerance + 0.05, direct_waypoint_slow_radius);
    const double max_angular_speed = std::max(0.01, std::fabs(direct_waypoint_max_angular_speed));
    const double min_turn_speed =
        std::min(max_angular_speed, std::max(0.0, std::fabs(direct_waypoint_min_turn_speed)));
    const double in_place_angle_threshold =
        std::max(0.2, std::min(M_PI, std::fabs(direct_waypoint_in_place_angle_threshold)));
    const double control_frequency = std::max(1.0, direct_waypoint_control_frequency);
    const double near_goal_heading_radius =
        std::max(goal_tolerance, std::fabs(direct_waypoint_near_goal_heading_radius));
    const double near_goal_max_speed =
        std::min(cruise_speed, std::max(0.0, std::fabs(direct_waypoint_near_goal_max_speed)));
    const double near_goal_max_angular_speed =
        std::min(max_angular_speed, std::max(0.0, std::fabs(direct_waypoint_near_goal_max_angular_speed)));

    bool marker_turned_green = false;
    double best_distance = 1e9;
    double closest_distance = 1e9;
    bool have_stable_desired_yaw = false;
    double stable_desired_yaw = 0.0;
    bool near_goal_overshoot_recovery = false;
    ros::Time last_progress_time = ros::Time::now();
    ros::Rate rate(control_frequency);

    ROS_INFO(
        "Direct waypoint drive started for waypoint %d/%d. Goal tolerance: %.2fm.",
        waypoint_index,
        total_waypoints,
        goal_tolerance);

    while(ros::ok())
    {
        ros::spinOnce();

        RobotPose2D robot_pose;
        if(!tryGetRobotPoseInFrame(listener, goal_frame, robot_pose))
        {
            publishStop(cmd_pub);
            rate.sleep();
            continue;
        }

        const double dx = waypoint.point.x - robot_pose.x;
        const double dy = waypoint.point.y - robot_pose.y;
        const double distance = std::sqrt((dx * dx) + (dy * dy));

        closest_distance = std::min(closest_distance, distance);
        if(!near_goal_overshoot_recovery &&
           closest_distance <= near_goal_heading_radius &&
           distance > (closest_distance + direct_waypoint_progress_epsilon))
        {
            near_goal_overshoot_recovery = true;
            have_stable_desired_yaw = false;
            ROS_WARN(
                "Waypoint %d/%d distance increased from %.3fm to %.3fm near goal; "
                "releasing heading hold and steering back to the waypoint.",
                waypoint_index,
                total_waypoints,
                closest_distance,
                distance);
        }

        if(!marker_turned_green && distance <= waypoint_marker_activation_radius)
        {
            publishReachedWaypointMarker(marker_pub, waypoint, waypoint_index);
            marker_turned_green = true;
            ROS_INFO(
                "Waypoint %d/%d entered marker activation radius (%.3f m <= %.3f m).",
                waypoint_index,
                total_waypoints,
                distance,
                waypoint_marker_activation_radius);
        }

        if(distance <= goal_tolerance)
        {
            if(!marker_turned_green)
            {
                publishReachedWaypointMarker(marker_pub, waypoint, waypoint_index);
            }
            publishStop(cmd_pub);
            ROS_INFO(
                "Direct waypoint drive reached waypoint %d/%d (%.3f m <= %.3f m).",
                waypoint_index,
                total_waypoints,
                distance,
                goal_tolerance);
            return true;
        }

        const ros::Time now = ros::Time::now();
        if((best_distance - distance) >= direct_waypoint_progress_epsilon)
        {
            best_distance = distance;
            last_progress_time = now;
        }

        if(direct_waypoint_progress_timeout > 0.0 &&
           (now - last_progress_time).toSec() > direct_waypoint_progress_timeout)
        {
            publishStop(cmd_pub);
            ROS_WARN(
                "Direct waypoint drive timed out for waypoint %d/%d. Best distance %.2fm, current distance %.2fm.",
                waypoint_index,
                total_waypoints,
                best_distance,
                distance);
            return false;
        }

        const bool near_goal_heading_hold =
            !near_goal_overshoot_recovery &&
            near_goal_heading_radius > goal_tolerance &&
            distance <= near_goal_heading_radius;
        if(!near_goal_heading_hold || !have_stable_desired_yaw)
        {
            stable_desired_yaw = std::atan2(dy, dx);
            have_stable_desired_yaw = true;
        }

        const double desired_yaw =
            near_goal_heading_hold ? stable_desired_yaw : std::atan2(dy, dx);
        const double yaw_error = normalizeAngle(desired_yaw - robot_pose.yaw);
        const double abs_yaw_error = std::fabs(yaw_error);

        geometry_msgs::Twist cmd;
        double angular_cmd = clampDouble(
            direct_waypoint_angular_kp * yaw_error,
            -max_angular_speed,
            max_angular_speed);

        if(abs_yaw_error > in_place_angle_threshold && !near_goal_heading_hold)
        {
            cmd.linear.x = 0.0;
            if(std::fabs(angular_cmd) < min_turn_speed)
            {
                angular_cmd = (yaw_error >= 0.0 ? 1.0 : -1.0) * min_turn_speed;
            }
        }
        else
        {
            double speed = cruise_speed;
            if(distance < slow_radius)
            {
                const double slow_scale = clampDouble(
                    (distance - goal_tolerance) / std::max(0.01, slow_radius - goal_tolerance),
                    0.0,
                    1.0);
                speed = min_speed + ((cruise_speed - min_speed) * slow_scale);
            }

            const double heading_scale = clampDouble(
                1.0 - (0.65 * abs_yaw_error / in_place_angle_threshold),
                0.25,
                1.0);
            cmd.linear.x = speed * heading_scale;
            if(near_goal_heading_hold && near_goal_max_speed > 0.0)
            {
                cmd.linear.x = std::min(cmd.linear.x, near_goal_max_speed);
            }
        }

        if(near_goal_heading_hold)
        {
            angular_cmd = clampDouble(
                angular_cmd,
                -near_goal_max_angular_speed,
                near_goal_max_angular_speed);
        }

        cmd.angular.z = angular_cmd;
        cmd_pub.publish(cmd);

        ROS_INFO_THROTTLE(
            1.0,
            "Direct waypoint %d/%d: dist=%.2fm bearing=%.1fdeg yaw=%.1fdeg err=%.1fdeg cmd=(%.2f, %.2f).",
            waypoint_index,
            total_waypoints,
            distance,
            desired_yaw * 180.0 / M_PI,
            robot_pose.yaw * 180.0 / M_PI,
            yaw_error * 180.0 / M_PI,
            cmd.linear.x,
            cmd.angular.z);

        rate.sleep();
    }

    publishStop(cmd_pub);
    return false;
}

bool driveDistanceInFrame(
    tf::TransformListener& listener,
    ros::Publisher& cmd_pub,
    const std::string& target_frame,
    double distance,
    double speed)
{
    if(std::fabs(distance) < 1e-6)
    {
        return true;
    }

    RobotPose2D start_pose;
    if(!tryGetRobotPoseInFrame(listener, target_frame, start_pose))
    {
        return false;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = (distance > 0.0 ? 1.0 : -1.0) * std::fabs(speed);
    const double target_distance = std::fabs(distance);
    const double timeout = std::max(4.0, (target_distance / std::max(0.05, std::fabs(speed))) * 3.0);

    ros::Rate rate(20.0);
    const ros::Time start_time = ros::Time::now();
    while(ros::ok() && ((ros::Time::now() - start_time).toSec() < timeout))
    {
        RobotPose2D current_pose;
        if(tryGetRobotPoseInFrame(listener, target_frame, current_pose))
        {
            const double dx = current_pose.x - start_pose.x;
            const double dy = current_pose.y - start_pose.y;
            if(std::sqrt((dx * dx) + (dy * dy)) >= target_distance)
            {
                publishStop(cmd_pub);
                return true;
            }
        }

        cmd_pub.publish(cmd);
        rate.sleep();
    }

    publishStop(cmd_pub);
    return false;
}

bool rotateAngleInFrame(
    tf::TransformListener& listener,
    ros::Publisher& cmd_pub,
    const std::string& target_frame,
    double angle_rad,
    double angular_speed)
{
    if(std::fabs(angle_rad) < 1e-6)
    {
        return true;
    }

    RobotPose2D start_pose;
    if(!tryGetRobotPoseInFrame(listener, target_frame, start_pose))
    {
        return false;
    }

    geometry_msgs::Twist cmd;
    cmd.angular.z = (angle_rad > 0.0 ? 1.0 : -1.0) * std::fabs(angular_speed);
    const double target_angle = std::fabs(angle_rad);
    const double timeout = std::max(3.0, (target_angle / std::max(0.05, std::fabs(angular_speed))) * 3.0);

    ros::Rate rate(20.0);
    const ros::Time start_time = ros::Time::now();
    while(ros::ok() && ((ros::Time::now() - start_time).toSec() < timeout))
    {
        RobotPose2D current_pose;
        if(tryGetRobotPoseInFrame(listener, target_frame, current_pose))
        {
            if(std::fabs(normalizeAngle(current_pose.yaw - start_pose.yaw)) >= target_angle)
            {
                publishStop(cmd_pub);
                return true;
            }
        }

        cmd_pub.publish(cmd);
        rate.sleep();
    }

    publishStop(cmd_pub);
    return false;
}

bool performOscillationRecovery(
    tf::TransformListener& listener,
    ros::Publisher& cmd_pub,
    int recovery_attempt_count)
{
    double turn_direction = 1.0;
    const bool have_scan_direction = chooseRecoveryTurnDirection(turn_direction);
    if((recovery_attempt_count % 2) == 0)
    {
        turn_direction *= -1.0;
    }

    const double turn_angle_rad = turn_direction * oscillation_recovery_turn_angle_deg * M_PI / 180.0;
    const char* turn_side = (turn_direction > 0.0) ? "left" : "right";

    ROS_WARN(
        "Executing oscillation recovery maneuver: sidestep %s by rotating %.1fdeg, driving %.2fm, then rotating back. Scan guidance: %s.",
        turn_side,
        oscillation_recovery_turn_angle_deg,
        oscillation_recovery_forward_distance,
        (have_scan_direction ? "yes" : "no"));

    publishStop(cmd_pub);
    ros::Duration(0.2).sleep();

    const bool rotated = rotateAngleInFrame(
        listener,
        cmd_pub,
        "odom",
        turn_angle_rad,
        oscillation_recovery_angular_speed);

    ros::Duration(0.2).sleep();

    const bool advanced = driveDistanceInFrame(
        listener,
        cmd_pub,
        "odom",
        oscillation_recovery_forward_distance,
        oscillation_recovery_linear_speed);

    ros::Duration(0.2).sleep();

    const bool rotated_back = rotateAngleInFrame(
        listener,
        cmd_pub,
        "odom",
        -turn_angle_rad,
        oscillation_recovery_angular_speed);

    publishStop(cmd_pub);
    ros::Duration(0.5).sleep();

    return rotated && advanced && rotated_back;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_waypoint"); //initiate node called gps_waypoint
    ros::NodeHandle n;
    int currentWaypointIndex = 0;
    int totalWaypoints = 0;
    int reachedWaypoints = 0;
    ROS_INFO("Initiated gps_waypoint node");
    MoveBaseClient ac("/move_base", true);
    tf::TransformListener tf_listener;
    //construct an action client that we use to communication with the action named move_base.
    //Setting true is telling the constructor to start ros::spin()

    // Initiate publisher to send end of node message
    ros::Publisher pubWaypointNodeEnded = n.advertise<std_msgs::Bool>("/outdoor_waypoint_nav/waypoint_following_status", 100);
    ros::Publisher pubWaypointReachedIndex = n.advertise<std_msgs::Int32>(
        "/outdoor_waypoint_nav/waypoint_reached_index",
        1,
        true);
    publishReachedWaypointIndex(pubWaypointReachedIndex, 0);
    ros::Publisher pubWaypointMarkers = n.advertise<visualization_msgs::Marker>(
        "/outdoor_waypoint_nav/collected_waypoints",
        100,
        true);
    ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_cmd_topic", waypoint_cmd_topic, "/cmd_vel");
    ros::Publisher pubRecoveryCmd = n.advertise<geometry_msgs::Twist>(waypoint_cmd_topic, 10);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        wait_count++;
        if(wait_count > 3)
        {
            ROS_ERROR("move_base action server did not come up, killing gps_waypoint node...");
            // Notify joy_launch_control that waypoint following is complete
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //Get Longitude and Latitude goals from text file

    //Count number of waypoints
    ros::param::get("/outdoor_waypoint_nav/coordinates_file", path_local);
    ros::param::param<std::string>("/outdoor_waypoint_nav/goal_frame", goal_frame, "map");
    ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_marker_frame", waypoint_marker_frame, goal_frame);
    ros::param::param<std::string>("/outdoor_waypoint_nav/waypoint_marker_topic", waypoint_marker_topic, "/outdoor_waypoint_nav/collected_waypoints");
    ros::param::param<double>("/outdoor_waypoint_nav/waypoint_marker_scale", waypoint_marker_scale, 0.7);
    ros::param::param<double>("/outdoor_waypoint_nav/waypoint_marker_activation_radius", waypoint_marker_activation_radius, 0.675);
    ros::param::param<double>("/outdoor_waypoint_nav/max_waypoint_goal_yaw_delta", max_waypoint_goal_yaw_delta, 0.60);
    ros::param::param<double>("/outdoor_waypoint_nav/waypoint_heading_preview_radius", waypoint_heading_preview_radius, 1.0);
    ros::param::param<double>("/outdoor_waypoint_nav/waypoint_advance_radius", waypoint_advance_radius, 0.0);
    ros::param::param<bool>("/outdoor_waypoint_nav/oscillation_recovery_enabled", oscillation_recovery_enabled, true);
    ros::param::param<int>("/outdoor_waypoint_nav/oscillation_recovery_max_attempts", oscillation_recovery_max_attempts, 2);
    ros::param::param<double>("/outdoor_waypoint_nav/oscillation_recovery_reverse_distance", oscillation_recovery_reverse_distance, 1.0);
    ros::param::param<double>("/outdoor_waypoint_nav/oscillation_recovery_turn_angle_deg", oscillation_recovery_turn_angle_deg, 90.0);
    ros::param::param<double>("/outdoor_waypoint_nav/oscillation_recovery_forward_distance", oscillation_recovery_forward_distance, 1.5);
    ros::param::param<double>("/outdoor_waypoint_nav/oscillation_recovery_linear_speed", oscillation_recovery_linear_speed, 0.30);
    ros::param::param<double>("/outdoor_waypoint_nav/oscillation_recovery_angular_speed", oscillation_recovery_angular_speed, 0.45);
    ros::param::param<bool>("/outdoor_waypoint_nav/safety_replan_enabled", safety_replan_enabled, true);
    ros::param::param<std::string>("/outdoor_waypoint_nav/safety_replan_topic", safety_replan_topic, "/outdoor_waypoint_nav/replan_requested");
    ros::param::param<double>("/outdoor_waypoint_nav/safety_replan_cooldown", safety_replan_cooldown, 0.0);
    ros::param::param<bool>("/outdoor_waypoint_nav/auto_replan_current_goal_enabled", auto_replan_current_goal_enabled, false);
    ros::param::param<double>("/outdoor_waypoint_nav/auto_replan_current_goal_interval", auto_replan_current_goal_interval, 1.5);
    ros::param::param<std::string>("/outdoor_waypoint_nav/preferred_avoidance_turn_topic", preferred_avoidance_turn_topic, "/outdoor_waypoint_nav/preferred_avoidance_turn_direction");
    ros::param::param<double>("/outdoor_waypoint_nav/preferred_avoidance_turn_deadband_deg", preferred_avoidance_turn_deadband_deg, 8.0);
    ros::param::param<double>("/outdoor_waypoint_nav/preferred_avoidance_turn_sign", preferred_avoidance_turn_sign, 1.0);
    ros::param::param<bool>("/outdoor_waypoint_nav/waypoint_pre_align_enabled", waypoint_pre_align_enabled, false);
    ros::param::param<double>("/outdoor_waypoint_nav/waypoint_pre_align_angle_threshold", waypoint_pre_align_angle_threshold, 0.35);
    ros::param::param<double>("/outdoor_waypoint_nav/waypoint_pre_align_min_goal_distance", waypoint_pre_align_min_goal_distance, 0.80);
    ros::param::param<bool>("/outdoor_waypoint_nav/return_to_start_enabled", return_to_start_enabled, true);
    ros::param::param<bool>("/outdoor_waypoint_nav/return_heading_pid_enabled", return_heading_pid_enabled, true);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_kp", return_heading_pid_kp, 1.4);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_ki", return_heading_pid_ki, 0.02);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_kd", return_heading_pid_kd, 0.08);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_tolerance_deg", return_heading_pid_tolerance_deg, 5.0);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_max_angular_speed", return_heading_pid_max_angular_speed, 0.8);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_min_angular_speed", return_heading_pid_min_angular_speed, 0.12);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_integral_limit", return_heading_pid_integral_limit, 1.0);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_settle_time", return_heading_pid_settle_time, 0.4);
    ros::param::param<double>("/outdoor_waypoint_nav/return_heading_pid_timeout", return_heading_pid_timeout, 12.0);
    ros::param::param<bool>("/outdoor_waypoint_nav/direct_waypoint_drive_enabled", direct_waypoint_drive_enabled, false);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_goal_tolerance", direct_waypoint_goal_tolerance, 0.10);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_final_goal_tolerance", direct_waypoint_final_goal_tolerance, 0.10);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_cruise_speed", direct_waypoint_cruise_speed, 0.45);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_min_speed", direct_waypoint_min_speed, 0.12);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_slow_radius", direct_waypoint_slow_radius, 2.0);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_angular_kp", direct_waypoint_angular_kp, 1.25);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_max_angular_speed", direct_waypoint_max_angular_speed, 0.60);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_min_turn_speed", direct_waypoint_min_turn_speed, 0.16);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_in_place_angle_threshold", direct_waypoint_in_place_angle_threshold, 1.55);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_progress_timeout", direct_waypoint_progress_timeout, 20.0);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_progress_epsilon", direct_waypoint_progress_epsilon, 0.05);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_control_frequency", direct_waypoint_control_frequency, 20.0);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_near_goal_heading_radius", direct_waypoint_near_goal_heading_radius, 0.0);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_near_goal_max_speed", direct_waypoint_near_goal_max_speed, 0.08);
    ros::param::param<double>("/outdoor_waypoint_nav/direct_waypoint_near_goal_max_angular_speed", direct_waypoint_near_goal_max_angular_speed, 0.15);
    if(waypoint_marker_topic != "/outdoor_waypoint_nav/collected_waypoints")
    {
        pubWaypointMarkers.shutdown();
        pubWaypointMarkers = n.advertise<visualization_msgs::Marker>(waypoint_marker_topic, 100, true);
    }
    ros::Publisher pubPreferredAvoidanceTurn =
        n.advertise<std_msgs::Float64>(preferred_avoidance_turn_topic, 10);
    ros::Subscriber subScan = n.subscribe("/front/scan", 20, scanCB);
    ros::Subscriber subSafetyReplan = n.subscribe(safety_replan_topic, 10, safetyReplanCB);
    numWaypoints = countWaypointsInFile(path_local);
    totalWaypoints = static_cast<int>(numWaypoints);
    ROS_INFO("Starting waypoint following for %d waypoint(s).", totalWaypoints);

    //Reading waypoints from text file and output results
    waypointVect = getWaypoints(path_local);

    double initial_robot_yaw = 0.0;
    bool have_initial_robot_yaw = waitForRobotYawInFrame(tf_listener, goal_frame, 5.0, initial_robot_yaw);
    if(have_initial_robot_yaw)
    {
        ROS_INFO("Captured initial robot yaw: %.1f deg.", initial_robot_yaw * 180.0 / M_PI);
    }
    else
    {
        ROS_WARN("Could not capture initial robot yaw before driving. Will retry before first goal.");
    }

    // Iterate through vector of waypoints for setting goals
    for(iter = waypointVect.begin(); iter < waypointVect.end(); iter++)
    {
        currentWaypointIndex++;

        //Setting goal:
        latiGoal = iter->first;
        longiGoal = iter->second;
        bool final_point = false;

        //set next goal point if not at last waypoint
        if(iter < (waypointVect.end() - 1))
        {
            iter++;
            latiNext = iter->first;
            longiNext = iter->second;
            iter--;
        }
        else //set to current
        {
            latiNext = iter->first;
            longiNext = iter->second;
            final_point = true;
        }

        ROS_INFO("Waypoint %d/%d selected", currentWaypointIndex, totalWaypoints);
        ROS_INFO("Waypoint %d/%d latitude: %.8f", currentWaypointIndex, totalWaypoints, latiGoal);
        ROS_INFO("Waypoint %d/%d longitude: %.8f", currentWaypointIndex, totalWaypoints, longiGoal);

        //Convert lat/long to utm:
        UTM_point = latLongtoUTM(latiGoal, longiGoal);
        UTM_next = latLongtoUTM(latiNext, longiNext);

        //Transform UTM to map point in odom frame
        map_point = UTMtoMapPoint(UTM_point);
        map_next = UTMtoMapPoint(UTM_next);

        ROS_INFO(
            "Waypoint %d/%d transformed in %s frame: x=%.3f y=%.3f",
            currentWaypointIndex,
            totalWaypoints,
            goal_frame.c_str(),
            map_point.point.x,
            map_point.point.y);

        alignTowardWaypointIfNeeded(
            tf_listener,
            pubRecoveryCmd,
            map_point,
            currentWaypointIndex,
            totalWaypoints);

        if(direct_waypoint_drive_enabled)
        {
            ac.cancelAllGoals();
            const bool reached_directly = driveDirectToWaypoint(
                tf_listener,
                pubRecoveryCmd,
                pubWaypointMarkers,
                map_point,
                currentWaypointIndex,
                totalWaypoints);

            if(reached_directly)
            {
                reachedWaypoints++;
                publishReachedWaypointIndex(pubWaypointReachedIndex, currentWaypointIndex);
                continue;
            }

            ROS_ERROR("Direct waypoint drive failed for waypoint %d/%d.", currentWaypointIndex, totalWaypoints);
            ROS_INFO("Exiting node...");
            std_msgs::Bool node_ended;
            // False means the controller stopped without completing the route.
            // Evaluators only finalize a run when this status is true.
            node_ended.data = false;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
            return 0;
        }

        bool have_prev_point = false;
        geometry_msgs::PointStamped map_prev;
        if(currentWaypointIndex > 1)
        {
            const double latiPrev = waypointVect[currentWaypointIndex - 2].first;
            const double longiPrev = waypointVect[currentWaypointIndex - 2].second;
            map_prev = UTMtoMapPoint(latLongtoUTM(latiPrev, longiPrev));
            have_prev_point = true;
        }

        bool waypoint_reached = false;
        bool marker_turned_green = false;
        int recovery_attempt_count = 0;
        while(ros::ok() && !waypoint_reached)
        {
            bool waypoint_reached_by_radius = false;
            double current_robot_yaw = 0.0;
            const bool have_current_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, current_robot_yaw);
            if(!have_initial_robot_yaw && have_current_yaw)
            {
                initial_robot_yaw = current_robot_yaw;
                have_initial_robot_yaw = true;
                ROS_INFO("Captured initial robot yaw before first goal: %.1f deg.", initial_robot_yaw * 180.0 / M_PI);
            }

            move_base_msgs::MoveBaseGoal goal = buildGoal(
                have_prev_point,
                map_prev,
                map_point,
                !final_point,
                map_next,
                final_point,
                false,
                have_current_yaw,
                current_robot_yaw);

            ROS_INFO("Sending waypoint %d/%d", currentWaypointIndex, totalWaypoints);
            ac.sendGoal(goal);
            last_auto_replan_time = ros::Time::now();

            bool preview_goal_sent = final_point;
            while(ros::ok())
            {
                ros::spinOnce();
                publishPreferredAvoidanceTurn(
                    pubPreferredAvoidanceTurn,
                    tf_listener,
                    map_point,
                    map_next,
                    final_point,
                    preview_goal_sent);

                geometry_msgs::PointStamped robot_point;
                if(tryGetRobotPointInFrame(tf_listener, waypoint_marker_frame, robot_point))
                {
                    const double dx = robot_point.point.x - map_point.point.x;
                    const double dy = robot_point.point.y - map_point.point.y;
                    const double distance_to_waypoint = std::sqrt((dx * dx) + (dy * dy));

                    if(!preview_goal_sent && distance_to_waypoint <= waypoint_heading_preview_radius)
                    {
                        double preview_robot_yaw = 0.0;
                        const bool have_preview_robot_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, preview_robot_yaw);
                        move_base_msgs::MoveBaseGoal preview_goal = buildGoal(
                            have_prev_point,
                            map_prev,
                            map_point,
                            !final_point,
                            map_next,
                            final_point,
                            true,
                            have_preview_robot_yaw,
                            preview_robot_yaw);
                        ac.sendGoal(preview_goal);
                        last_auto_replan_time = ros::Time::now();
                        preview_goal_sent = true;
                        ROS_INFO(
                            "Waypoint %d/%d entered heading preview radius (%.3f m <= %.3f m).",
                            currentWaypointIndex,
                            totalWaypoints,
                            distance_to_waypoint,
                            waypoint_heading_preview_radius);
                    }

                    if(!marker_turned_green && distance_to_waypoint <= waypoint_marker_activation_radius)
                    {
                        publishReachedWaypointMarker(pubWaypointMarkers, map_point, currentWaypointIndex);
                        marker_turned_green = true;
                        ROS_INFO(
                            "Waypoint %d/%d entered marker activation radius (%.3f m <= %.3f m).",
                            currentWaypointIndex,
                            totalWaypoints,
                            distance_to_waypoint,
                            waypoint_marker_activation_radius);
                    }

                    const bool advance_to_next_waypoint = !final_point;
                    const bool finish_final_waypoint =
                        final_point && (totalWaypoints == 1 || !return_to_start_enabled);
                    const bool handoff_final_waypoint_to_return =
                        final_point && totalWaypoints > 1 && return_to_start_enabled;
                    if(waypoint_advance_radius > 0.0 &&
                       distance_to_waypoint <= waypoint_advance_radius &&
                       (advance_to_next_waypoint || finish_final_waypoint || handoff_final_waypoint_to_return))
                    {
                        waypoint_reached_by_radius = true;
                        if(finish_final_waypoint)
                        {
                            ROS_INFO(
                                "Final waypoint %d/%d entered advance radius (%.3f m <= %.3f m). "
                                "Finishing without waiting for move_base SUCCEEDED.",
                                currentWaypointIndex,
                                totalWaypoints,
                                distance_to_waypoint,
                                waypoint_advance_radius);
                            ac.cancelAllGoals();
                            publishStop(pubRecoveryCmd);
                        }
                        else if(handoff_final_waypoint_to_return)
                        {
                            ROS_INFO(
                                "Last waypoint %d/%d entered advance radius (%.3f m <= %.3f m). "
                                "Handing off to return-to-start goal without waiting for SUCCEEDED.",
                                currentWaypointIndex,
                                totalWaypoints,
                                distance_to_waypoint,
                                waypoint_advance_radius);
                        }
                        else
                        {
                            ROS_INFO(
                                "Waypoint %d/%d entered advance radius (%.3f m <= %.3f m). "
                                "Sending the next waypoint without waiting for SUCCEEDED.",
                                currentWaypointIndex,
                                totalWaypoints,
                                distance_to_waypoint,
                                waypoint_advance_radius);
                        }
                    }
                }

                if(waypoint_reached_by_radius)
                {
                    break;
                }

                if(consumeSafetyReplanRequest())
                {
                    double replan_robot_yaw = 0.0;
                    const bool have_replan_robot_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, replan_robot_yaw);
                    move_base_msgs::MoveBaseGoal replanned_goal = buildGoal(
                        have_prev_point,
                        map_prev,
                        map_point,
                        !final_point,
                        map_next,
                        final_point,
                        preview_goal_sent,
                        have_replan_robot_yaw,
                        replan_robot_yaw);
                    ac.sendGoal(replanned_goal);
                    last_auto_replan_time = ros::Time::now();
                    ROS_WARN(
                        "Safety avoidance requested replan. Resent waypoint %d/%d so move_base replans from the current robot pose.",
                        currentWaypointIndex,
                        totalWaypoints);
                }
                else if(shouldAutoReplanCurrentGoal())
                {
                    double replan_robot_yaw = 0.0;
                    const bool have_replan_robot_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, replan_robot_yaw);
                    move_base_msgs::MoveBaseGoal refreshed_goal = buildGoal(
                        have_prev_point,
                        map_prev,
                        map_point,
                        !final_point,
                        map_next,
                        final_point,
                        preview_goal_sent,
                        have_replan_robot_yaw,
                        replan_robot_yaw);
                    ac.sendGoal(refreshed_goal);
                    ROS_INFO_THROTTLE(
                        5.0,
                        "Refreshing waypoint %d/%d goal so move_base redraws the plan from the current robot pose.",
                        currentWaypointIndex,
                        totalWaypoints);
                }

                if(ac.waitForResult(ros::Duration(0.1)))
                {
                    break;
                }
            }

            if(waypoint_reached_by_radius)
            {
                reachedWaypoints++;
                publishReachedWaypointIndex(pubWaypointReachedIndex, currentWaypointIndex);
                if(!marker_turned_green)
                {
                    publishReachedWaypointMarker(pubWaypointMarkers, map_point, currentWaypointIndex);
                }
                waypoint_reached = true;
                break;
            }

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                reachedWaypoints++;
                publishReachedWaypointIndex(pubWaypointReachedIndex, currentWaypointIndex);
                if(!marker_turned_green)
                {
                    publishReachedWaypointMarker(pubWaypointMarkers, map_point, currentWaypointIndex);
                }
                ROS_INFO("Reached waypoint %d/%d", currentWaypointIndex, totalWaypoints);
                waypoint_reached = true;
                break;
            }

            const std::string move_base_text = ac.getState().getText();
            const bool oscillation_abort =
                (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) &&
                (move_base_text.find("oscillat") != std::string::npos);

            if(oscillation_recovery_enabled &&
               oscillation_abort &&
               recovery_attempt_count < oscillation_recovery_max_attempts)
            {
                recovery_attempt_count++;
                ROS_WARN(
                    "Waypoint %d/%d aborted due to oscillation. Starting custom recovery attempt %d/%d.",
                    currentWaypointIndex,
                    totalWaypoints,
                    recovery_attempt_count,
                    oscillation_recovery_max_attempts);
                ac.cancelAllGoals();
                const bool recovery_ok = performOscillationRecovery(tf_listener, pubRecoveryCmd, recovery_attempt_count);
                if(recovery_ok)
                {
                    ROS_INFO("Custom oscillation recovery finished. Retrying waypoint %d/%d.", currentWaypointIndex, totalWaypoints);
                    continue;
                }
                ROS_WARN("Custom oscillation recovery did not complete cleanly. Falling back to normal failure handling.");
            }

            ROS_ERROR(
                "Failed to reach waypoint %d/%d. move_base state: %s. %s",
                currentWaypointIndex,
                totalWaypoints,
                ac.getState().toString().c_str(),
                move_base_text.c_str());
            ROS_ERROR("GPS waypoint unreachable.");
            ROS_INFO("Exiting node...");
            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }
    } // End for loop iterating through waypoint vector

    if(return_to_start_enabled && totalWaypoints > 1 && reachedWaypoints == totalWaypoints)
    {
        const double latiReturn = waypointVect.front().first;
        const double longiReturn = waypointVect.front().second;
        const double latiReturnNext = waypointVect[1].first;
        const double longiReturnNext = waypointVect[1].second;

        ROS_INFO("Completed waypoint list. Returning to waypoint 1/%d before finishing.", totalWaypoints);

        const geometry_msgs::PointStamped map_return_prev = map_point;
        UTM_point = latLongtoUTM(latiReturn, longiReturn);
        UTM_next = latLongtoUTM(latiReturnNext, longiReturnNext);
        map_point = UTMtoMapPoint(UTM_point);
        map_next = UTMtoMapPoint(UTM_next);

        ROS_INFO(
            "Return waypoint 1/%d transformed in %s frame: x=%.3f y=%.3f",
            totalWaypoints,
            goal_frame.c_str(),
            map_point.point.x,
            map_point.point.y);

        int return_recovery_attempt_count = 0;
        bool returned_to_start = false;
        while(ros::ok())
        {
            double current_robot_yaw = 0.0;
            const bool have_current_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, current_robot_yaw);
            move_base_msgs::MoveBaseGoal return_goal = buildGoal(
                true,
                map_return_prev,
                map_point,
                true,
                map_next,
                false,
                false,
                have_current_yaw,
                current_robot_yaw);
            ROS_INFO("Sending return goal to waypoint 1/%d", totalWaypoints);
            ac.sendGoal(return_goal);
            last_auto_replan_time = ros::Time::now();
            while(ros::ok())
            {
                ros::spinOnce();
                publishPreferredAvoidanceTurn(
                    pubPreferredAvoidanceTurn,
                    tf_listener,
                    map_point,
                    map_next,
                    false,
                    false);

                if(consumeSafetyReplanRequest())
                {
                    double replan_robot_yaw = 0.0;
                    const bool have_replan_robot_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, replan_robot_yaw);
                    move_base_msgs::MoveBaseGoal replanned_return_goal = buildGoal(
                        true,
                        map_return_prev,
                        map_point,
                        true,
                        map_next,
                        false,
                        false,
                        have_replan_robot_yaw,
                        replan_robot_yaw);
                    ac.sendGoal(replanned_return_goal);
                    last_auto_replan_time = ros::Time::now();
                    ROS_WARN(
                        "Safety avoidance requested replan. Resent return goal to waypoint 1/%d from the current robot pose.",
                        totalWaypoints);
                }
                else if(shouldAutoReplanCurrentGoal())
                {
                    double replan_robot_yaw = 0.0;
                    const bool have_replan_robot_yaw = tryGetRobotYawInFrame(tf_listener, goal_frame, replan_robot_yaw);
                    move_base_msgs::MoveBaseGoal refreshed_return_goal = buildGoal(
                        true,
                        map_return_prev,
                        map_point,
                        true,
                        map_next,
                        false,
                        false,
                        have_replan_robot_yaw,
                        replan_robot_yaw);
                    ac.sendGoal(refreshed_return_goal);
                    ROS_INFO_THROTTLE(
                        5.0,
                        "Refreshing return goal to waypoint 1/%d so move_base redraws the plan from the current robot pose.",
                        totalWaypoints);
                }

                if(ac.waitForResult(ros::Duration(0.1)))
                {
                    break;
                }
            }

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Returned to waypoint 1/%d", totalWaypoints);
                returned_to_start = true;
                break;
            }

            const std::string move_base_text = ac.getState().getText();
            const bool oscillation_abort =
                (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) &&
                (move_base_text.find("oscillat") != std::string::npos);

            if(oscillation_recovery_enabled &&
               oscillation_abort &&
               return_recovery_attempt_count < oscillation_recovery_max_attempts)
            {
                return_recovery_attempt_count++;
                ROS_WARN(
                    "Return to waypoint 1/%d aborted due to oscillation. Starting custom recovery attempt %d/%d.",
                    totalWaypoints,
                    return_recovery_attempt_count,
                    oscillation_recovery_max_attempts);
                ac.cancelAllGoals();
                const bool recovery_ok = performOscillationRecovery(tf_listener, pubRecoveryCmd, return_recovery_attempt_count);
                if(recovery_ok)
                {
                    ROS_INFO("Custom oscillation recovery finished. Retrying return to waypoint 1/%d.", totalWaypoints);
                    continue;
                }
                ROS_WARN("Custom oscillation recovery did not complete cleanly during return-to-start. Falling back to normal failure handling.");
            }

            ROS_ERROR(
                "Failed to return to waypoint 1/%d. move_base state: %s. %s",
                totalWaypoints,
                ac.getState().toString().c_str(),
                move_base_text.c_str());
            ROS_ERROR("GPS waypoint return-to-start unreachable.");
            ROS_INFO("Exiting node...");

            std_msgs::Bool node_ended;
            node_ended.data = true;
            pubWaypointNodeEnded.publish(node_ended);
            ros::shutdown();
        }

        if(!returned_to_start)
        {
            return 0;
        }

        if(return_heading_pid_enabled)
        {
            if(have_initial_robot_yaw)
            {
                ac.cancelAllGoals();
                publishStop(pubRecoveryCmd);
                ros::Duration(0.2).sleep();
                const bool heading_aligned = alignYawToTargetPID(
                    tf_listener,
                    pubRecoveryCmd,
                    goal_frame,
                    initial_robot_yaw);
                if(!heading_aligned)
                {
                    ROS_WARN("Return heading alignment did not fully converge, but waypoint following will finish.");
                }
            }
            else
            {
                ROS_WARN("Return heading PID skipped because the initial robot yaw was not available.");
            }
        }
    }

    ROS_INFO("Final status: reached %d/%d waypoint(s).", reachedWaypoints, totalWaypoints);
    if(return_to_start_enabled && totalWaypoints > 1)
    {
        ROS_INFO("Waypoint following complete: %d/%d waypoint(s) reached, returned to waypoint 1.", reachedWaypoints, totalWaypoints);
    }
    else
    {
        ROS_INFO("Waypoint following complete: %d/%d waypoint(s) reached.", reachedWaypoints, totalWaypoints);
    }
    ROS_INFO("Warthog has reached all of its goals!!!\n");
    ROS_INFO("Ending node...");

    // Notify joy_launch_control that waypoint following is complete
    std_msgs::Bool node_ended;
    node_ended.data = true;
    pubWaypointNodeEnded.publish(node_ended);

    ros::shutdown();
    ros::spin();
    return 0;
}
