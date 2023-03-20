// Code based on moveit_tutorial with several changes
// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp
// --------------------------------------------------

#include <gripper_ntlab_msgs/CartesianPosition.h>
#include <gripper_ntlab_msgs/JointPosition.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define BUFFER_MESSAGE 5

double finger_torque[2] = {0};

typedef std::map<std::string, std::vector<double>> Map;
gripper_ntlab_msgs::JointPosition gripper_joint_pos;
gripper_ntlab_msgs::CartesianPosition gripper_cartesian_pos;

Map m, cobotta_pose;

void insertPosition() {
    // Value of gripper use Inverse Kinematics, x1, y1, x2, y2, rad
    m.insert(Map::value_type("standby", std::vector<double>{0.19, -0.06, 0.19, 0.06, 0}));
    m.insert(Map::value_type("prepare_gripping", std::vector<double>{0.235, -0.05, 0.235, 0.05, 0}));
    m.insert(Map::value_type("grip", std::vector<double>{0.235, -0.02, 0.235, 0.02, 0}));
    m.insert(Map::value_type("tight_grip", std::vector<double>{0.237, -0.014, 0.237, 0.014, 0}));

    // Cobotta x, y, z, rw, rx, ry, rz
    cobotta_pose.insert(Map::value_type("standby", std::vector<double>{-0.00317901, 0.120532, 0.282123, 0.0134343, -0.755252, 0.655107, -0.0157766}));
    cobotta_pose.insert(Map::value_type("prepare_gripping", std::vector<double>{-0.00317901, 0.120532, 0.282123, 0.00704367, -0.754316, 0.656014, -0.0245619}));
    cobotta_pose.insert(Map::value_type("to_grip", std::vector<double>{-0.00317901, 0.120532, 0.282123, 0.00740522, -0.742819, 0.669006, -0.0244086}));
    cobotta_pose.insert(Map::value_type("rotate", std::vector<double>{-0.00317901, 0.177077, 0.313007, 0.00734797, -0.742775, 0.669057, -0.0243666}));
    cobotta_pose.insert(Map::value_type("place", std::vector<double>{-0.00317901, 0.206146, 0.282123, 0.00900081, -0.734338, 0.678259, -0.0251244}));
    cobotta_pose.insert(Map::value_type("post_place", std::vector<double>{-0.00317901, 0.120532, 0.282123, 0.00734797, -0.742775, 0.669057, -0.0243666}));
}

void gripperSetPose(std::vector<double> pos) {
    gripper_ntlab_msgs::CartesianPosition msg;
    msg.torque = true;
    msg.x1 = pos[0];
    msg.y1 = pos[1];
    msg.x2 = pos[2];
    msg.y2 = pos[3];
    msg.rad = pos[4];
    gripper_cartesian_pos = msg;
}

void gripperSetPose(std::string key) {
    Map::const_iterator pos = m.find(key);
    gripperSetPose(pos->second);
}

std::string printRobotPose(std::string msg, geometry_msgs::Pose pose) {
    std::stringstream ss;
    ss << msg << " ";
    ss << pose.position.x << ", ";
    ss << pose.position.y << ", ";
    ss << pose.position.z << ", ";
    ss << pose.orientation.w << ", ";
    ss << pose.orientation.x << ", ";
    ss << pose.orientation.y << ", ";
    ss << pose.orientation.z << "";
    return ss.str();
}

geometry_msgs::Pose setTarget(double px, double py, double pz, double ow, double ox, double oy, double oz) {
    geometry_msgs::Pose target;
    target.position.x = px;
    target.position.y = py;
    target.position.z = pz;

    target.orientation.w = ow;
    target.orientation.x = ox;
    target.orientation.y = oy;
    target.orientation.z = oz;

    return target;
}

geometry_msgs::Pose setPose(std::string key) {
    Map::const_iterator pos = cobotta_pose.find(key);
    return setTarget(pos->second[0], pos->second[1], pos->second[2], pos->second[3], pos->second[4], pos->second[5], pos->second[6]);
}

void cobottaJointSubCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    int pos = 0;
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "l_hand_rod_a") {
            finger_torque[0] = msg->effort[i];
        }

        if (msg->name[i] == "r_hand_rod_a") {
            finger_torque[1] = msg->effort[i];
        }
    }
    //ROS_INFO("Received: %1.3f, %1.3f", finger_torque[0], finger_torque[1]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_in_hand_manipulation");
    ros::NodeHandle node_handle;

    ros::Rate rate(100);

    ros::Subscriber cobotta_joint_state = node_handle.subscribe("cobotta/all_joint_states", 10, cobottaJointSubCallback);
    ros::Publisher gripper_pub = node_handle.advertise<gripper_ntlab_msgs::JointPosition>("gripper_ntlab/set_position", 10);
    ros::Publisher gripper_cartesian_pub = node_handle.advertise<gripper_ntlab_msgs::CartesianPosition>("cobotta/hand_set_cartesian", 10);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set the planning group that will be used
    static const std::string PLANNING_GROUP = "arm";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Set Parameters
    move_group.setMaxVelocityScalingFactor(0.7);
    move_group.setMaxAccelerationScalingFactor(0.7);

    // Visualization
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = -0.075;
    visual_tools.publishText(text_pose, "ntlab", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("denso_robot_move", "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("denso_robot_move", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    std::string planning_groups;
    ROS_INFO_NAMED("denso_robot_move", "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));
    ROS_INFO("\n");

    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    move_group.setPoseReferenceFrame("base_link");

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    insertPosition();

    // Set waypoints to display the way point in rviz
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose.pose);

    // Initialize Pose
    geometry_msgs::Pose target_pose;
    std::string step;

    // STAND BY ----------------------------------------------------------------------------------------------------------------------------------
    step = "standby";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // set gripper
    gripperSetPose(step);
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        // gripper_pub.publish(gripper_joint_pos);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        rate.sleep();
    }

    // move cobotta
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);
    ros::Duration(1.0).sleep();

    // Prepare for gripping ----------------------------------------------------------------------------------------------------------------------
    step = "prepare_gripping";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // set gripper
    gripperSetPose(step);
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        // gripper_pub.publish(gripper_joint_pos);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        rate.sleep();
    }

    // move cobotta
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);
    ros::Duration(1.0).sleep();

    // To grip --------------------------------------------------------------------------------------------------------------------------------
    step = "to_grip";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // move cobotta
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);
    ros::Duration(1.0).sleep();

    // GRIP ---------------------------------------------------------------------------------------------------------------------------------
    // step = "grip";
    // gripperSetPose(step);
    // for (int i = 0; i < BUFFER_MESSAGE; i++) {
    //     // gripper_pub.publish(gripper_joint_pos);
    //     gripper_cartesian_pub.publish(gripper_cartesian_pos);
    //     rate.sleep();
    // }
    double limit_torque = 0.048;
    std::vector<double> gripper_pose = {0.235, -0.05, 0.235, 0.05, 0};

    while (finger_torque[0] <= limit_torque &&
           finger_torque[0] >= -limit_torque &&
           finger_torque[1] <= limit_torque &&
           finger_torque[1] >= -limit_torque) {
        gripper_pose[1] += 0.0001;
        gripper_pose[3] -= 0.0001;
        gripperSetPose(gripper_pose);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        ros::Duration(0.01).sleep();
    }

    ros::Duration(1.0).sleep();

    // step = "tight_grip";
    // gripperSetPose(step);
    // for (int i = 0; i < BUFFER_MESSAGE; i++) {
    //     // gripper_pub.publish(gripper_joint_pos);
    //     gripper_cartesian_pub.publish(gripper_cartesian_pos);
    //     rate.sleep();
    // }

    ros::Duration(1.0).sleep();

    m.insert(Map::value_type("rotate", std::vector<double>{0.235, -0.014, 0.235, 0.014, 3.14}));
    m.insert(Map::value_type("prepare_place", std::vector<double>{0.235, -0.02, 0.235, 0.02, 3.14}));
    m.insert(Map::value_type("place", std::vector<double>{0.235, -0.03, 0.235, 0.03, 3.14}));

    // ROTATE --------------------------------------------------------------------------------------------------------------------------------
    step = "rotate";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // move cobotta
    move_group.move();

    gripperSetPose(step);
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        // gripper_pub.publish(gripper_joint_pos);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        rate.sleep();
    }

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);
    ros::Duration(1.0).sleep();

    // PLACE --------------------------------------------------------------------------------------------------------------------------------
    step = "place";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // move cobotta
    move_group.move();

    gripperSetPose("prepare_place");
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        // gripper_pub.publish(gripper_joint_pos);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        rate.sleep();
    }
    ros::Duration(1.0).sleep();

    gripperSetPose(step);
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        // gripper_pub.publish(gripper_joint_pos);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        rate.sleep();
    }
    ros::Duration(1.0).sleep();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);
    ros::Duration(1.0).sleep();

    // POST PLACE --------------------------------------------------------------------------------------------------------------------------------
    step = "post_place";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    gripperSetPose("standby");
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        // gripper_pub.publish(gripper_joint_pos);
        gripper_cartesian_pub.publish(gripper_cartesian_pos);
        rate.sleep();
    }

    // move cobotta
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    step = "standby";
    target_pose = setPose(step);
    move_group.setPoseTarget(target_pose);
    waypoints.push_back(target_pose);
    ROS_INFO_STREAM(printRobotPose("[TARGET POSE]", target_pose));

    ROS_INFO_STREAM("Visualizing plan '" << step << "' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose, step);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // move cobotta
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM(printRobotPose("[ROBOT POSE]", current_pose.pose));

    waypoints.clear();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish");

    // finish
    ros::shutdown();
    return 0;
}
