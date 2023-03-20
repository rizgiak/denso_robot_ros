// Code based on moveit_tutorial with several changes
// https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_interface/src/move_group_interface_tutorial.cpp
// --------------------------------------------------

#include <gripper_ntlab_msgs/JointPosition.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define BUFFER_MESSAGE 10

typedef std::map<std::string, std::vector<int>> Map;
gripper_ntlab_msgs::JointPosition gripper_joint_pos;

Map m;

void insertPosition() {
    m.insert(Map::value_type("standby", std::vector<int>{0, 0, 0, 0, 0}));
    m.insert(Map::value_type("rotate_finger", std::vector<int>{0, 0, 820, -760, 2000}));
    m.insert(Map::value_type("prepare_gripping", std::vector<int>{0, 0, 432, -540, 0}));
    m.insert(Map::value_type("prepare_place", std::vector<int>{0, 0, 790, -730, 2000}));
    m.insert(Map::value_type("place", std::vector<int>{0, 0, 700, -640, 2000}));
    m.insert(Map::value_type("grip", std::vector<int>{0, 0, 700, -660, 0}));
    m.insert(Map::value_type("tight_grip", std::vector<int>{0, 0, 120, -120, 0}));

    Map::const_iterator pos = m.find("rotate_finger");
    std::vector<int> position = pos->second;
}

void setPosition(std::vector<int> pos) {
    gripper_ntlab_msgs::JointPosition msg;
    msg.mode = 101;
    for (int i = 0; i < pos.size(); i++) {
        msg.position.push_back(pos[i]);
    }
    gripper_joint_pos = msg;
}

void rotateFinger() {
    // std::vector<int> pos = {0, 0, 820, -760, 2000};
    Map::const_iterator pos = m.find("rotate_finger");
    setPosition(pos->second);
}

void prepareGripping() {
    // std::vector<int> pos = {0, 0, 432, -540, 0};
    Map::const_iterator pos = m.find("prepare_gripping");
    setPosition(pos->second);
}

void preparePlace() {
    // std::vector<int> pos = {0, 0, 700, -640, 0};
    Map::const_iterator pos = m.find("prepare_place");
    setPosition(pos->second);
}

void place() {
    // std::vector<int> pos = {0, 0, 700, -640, 0};
    Map::const_iterator pos = m.find("place");
    setPosition(pos->second);
}

void grip() {
    // std::vector<int> pos = {0, 0, 790, -730, 0};
    Map::const_iterator pos = m.find("grip");
    setPosition(pos->second);
}

void tightGrip() {
    // std::vector<int> pos = {0, 0, 820, -760, 0};
    Map::const_iterator pos = m.find("tight_grip");
    setPosition(pos->second);
    gripper_joint_pos.mode = 124;
}

void standby() {
    Map::const_iterator pos = m.find("standby");
    setPosition(pos->second);
}

std::string printRobotPose(geometry_msgs::Pose pose) {
    std::stringstream ss;
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_in_hand_manipulation");
    ros::NodeHandle node_handle;

    ros::Rate rate(100);

    ros::Publisher gripper_pub = node_handle.advertise<gripper_ntlab_msgs::JointPosition>("gripper_ntlab/set_position", 10);

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
    ROS_INFO_STREAM("[ROBOT POSE] " + printRobotPose(current_pose.pose));

    move_group.setPoseReferenceFrame("base_link");

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    insertPosition();

    // Set waypoints to display the way point in rviz
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(current_pose.pose);

    // STAND BY ----------------------------------------------------------------------------------------------------------------------------------
    //  Planning to a Pose goal 1
    geometry_msgs::Pose target_pose1;
    target_pose1 = setTarget(0.00475721, 0.113944, 0.282596, 0.00708303, -0.754352, 0.655969, -0.0246456);
    move_group.setPoseTarget(target_pose1);
    waypoints.push_back(target_pose1);
    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // Set plan
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'standby' as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [STANDBY PHASE]");

    // set standby gripper
    standby();
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    // Moving to a pose goal
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE] " + printRobotPose(current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start [PREPARE TO GRIP]");
    ros::Duration(1.0).sleep();

    // Prepare to grip --------------------------------------------------------------------------------------------------------------------------
    target_pose1 = setTarget(0.00426273, 0.122024, 0.267944, 0.00704367, -0.754316, 0.656014, -0.0245619);
    move_group.setPoseTarget(target_pose1);
    waypoints.push_back(target_pose1);

    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'Prepare to grip' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose2");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [PREPARE TO GRIP]");

    prepareGripping();
    for (int i = 0; i < BUFFER_MESSAGE; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    // Moving to a pose goal
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE] " + printRobotPose(current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start [TO GRIP]");
    ros::Duration(1.0).sleep();

    // To grip --------------------------------------------------------------------------------------------------------------------------------
    target_pose1 = setTarget(0.00453416, 0.123062, 0.25627, 0.00740522, -0.742819, 0.669006, -0.0244086);
    move_group.setPoseTarget(target_pose1);
    waypoints.push_back(target_pose1);

    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'To grip' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose3");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [TO GRIP]");

    // Moving to a pose goal
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE] " + printRobotPose(current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start [GRIP]");
    ros::Duration(1.0).sleep();

    // GRIP ---------------------------------------------------------------------------------------------------------------------------------
    grip();
    for (int i = 0; i < 5; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    ros::Duration(1.0).sleep();

    tightGrip();
    for (int i = 0; i < 1; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start [ROTATE]");
    ros::Duration(3.0).sleep();

    // ROTATE --------------------------------------------------------------------------------------------------------------------------------
    target_pose1 = setTarget(-0.00355439, 0.177077, 0.313007, 0.00734797, -0.742775, 0.669057, -0.0243666);
    move_group.setPoseTarget(target_pose1);
    waypoints.push_back(target_pose1);

    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'rotate' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose4");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [ROTATE]");

    // Moving to a pose goal
    move_group.move();

    rotateFinger();
    for (int i = 0; i < 5; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE] " + printRobotPose(current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start [PLACE]");
    ros::Duration(1.0).sleep();

    // PLACE --------------------------------------------------------------------------------------------------------------------------------
    target_pose1 = setTarget(-0.00519238, 0.214783, 0.256939, 0.00900081, -0.734338, 0.678259, -0.0251244);
    move_group.setPoseTarget(target_pose1);
    waypoints.push_back(target_pose1);

    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'place' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose5");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [PLACE]");

    // Moving to a pose goal
    move_group.move();

    preparePlace();
    for (int i = 0; i < 5; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }
    ros::Duration(1.0).sleep();

    place();
    for (int i = 0; i < 5; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    ros::Duration(1.0).sleep();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE]  " + printRobotPose(current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start [STANDBY]");
    ros::Duration(1.0).sleep();

    // STANDBY --------------------------------------------------------------------------------------------------------------------------------
    target_pose1 = setTarget(-0.00355439, 0.177077, 0.313007, 0.00734797, -0.742775, 0.669057, -0.0243666);
    move_group.setPoseTarget(target_pose1);
    waypoints.push_back(target_pose1);

    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'back to standby' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose2");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [STANDBY]");

    // Moving to a pose goal
    move_group.move();

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE]  " + printRobotPose(current_pose.pose));

    waypoints.clear();
    waypoints.push_back(current_pose.pose);

    target_pose1 = setTarget(0.00475721, 0.113944, 0.282596, 0.00708303, -0.754352, 0.655969, -0.0246456);
    move_group.setPoseTarget(target_pose1);

    waypoints.push_back(target_pose1);

    ROS_INFO_STREAM("[TARGET POSE] " + printRobotPose(target_pose1));

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("denso_robot_move", "Visualizing plan 'standby' as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(target_pose1, "pose2");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute [STANDBY]");

    // Moving to a pose goal
    move_group.move();

    standby();
    for (int i = 0; i < 5; i++) {
        gripper_pub.publish(gripper_joint_pos);
        rate.sleep();
    }

    // Show current pose of robot.
    current_pose = move_group.getCurrentPose();
    ROS_INFO_STREAM("[ROBOT POSE]  " + printRobotPose(current_pose.pose));

    waypoints.clear();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish");

    // finish
    ros::shutdown();
    return 0;
}
