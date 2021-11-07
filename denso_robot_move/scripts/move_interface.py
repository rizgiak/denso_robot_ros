from __future__ import print_function

# from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from sensor_msgs.msg import JointState
from gripper_ntlab_controller.msg import CartesianPosition
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def allClose(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return allClose(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class GripperNTLab(object):

    gripper_pose = CartesianPosition()

    def __init__(self) -> None:
        super(GripperNTLab, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("move_interface", anonymous=True)
        rospy.Subscriber(
            "cobotta/all_joint_states", JointState, self.jointStateCallback
        )
        gripper_pub = rospy.Publisher(
            "cobotta/hand_set_cartesian", CartesianPosition, queue_size=10
        )

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        move_group.set_max_velocity_scaling_factor(0.7)
        move_group.set_max_acceleration_scaling_factor(0.7)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.gripper_pub = gripper_pub
        self.finger_torque = [0, 0]

    def cobottaExecutePoseGoal(self, position):

        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        pose_goal.orientation.w = position[3]
        pose_goal.orientation.x = position[4]
        pose_goal.orientation.y = position[5]
        pose_goal.orientation.z = position[6]

        move_group.set_pose_target(pose_goal)

        # call planner to execute
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return allClose(pose_goal, current_pose, 0.01)

    def jointStateCallback(self, data):
        i = 0
        f1, f2 = (0, 0)
        for t in data.name:
            if t == "l_hand_rod_a":
                f1 = data.effort[i]
            elif t == "r_hand_rod_b":
                f2 = data.effort[i]
            i += 1
        # rospy.loginfo("f1:{}, f2:{}".format(f1, f2))
        self.finger_torque = [f1, f2]

    def gripperSetPose(self, position):
        self.gripper_pose.x1 = position[0]
        self.gripper_pose.y1 = position[1]
        self.gripper_pose.x2 = position[2]
        self.gripper_pose.y2 = position[3]
        self.gripper_pose.rad = position[4]
        self.gripper_pose.torque = True

    def gripperExecute(self):
        gripper_pub = self.gripper_pub
        gripper_pub.publish(self.gripper_pose)

    def gripperGripLimitTorque(self):
        limit_torque = 0.048
        gripper_position = [0.235, -0.05, 0.235, 0.05, 0]
        print("f1:" + str(self.finger_torque[0]) + ", f2:" + str(self.finger_torque[1]))
        while (
            self.finger_torque[0] <= limit_torque
            and self.finger_torque[0] >= -limit_torque
            and self.finger_torque[1] <= limit_torque
            and self.finger_torque[1] >= -limit_torque
        ):
            gripper_position[1] += 0.0001
            gripper_position[3] -= 0.0001
            self.gripperSetPose(gripper_position)
            self.gripperExecute()
            print("res f1:" + str(self.finger_torque[0]) + ", f2:" + str(self.finger_torque[1]))
            rospy.sleep(0.01)


def main():
    try:
        print(sys.version)
        print("Move Interface")
        input("Press 'Enter' to start.") # standby
        ntlab = GripperNTLab()
        position = [
            0.00206299,
            0.148964,
            0.372349,
            0.0123763,
            -0.74677,
            0.664963,
            0.00219526,
        ]
        ntlab.cobottaExecutePoseGoal(position)

        input("Press 'Enter' to next.")
        gripper_pos = [0.22, -0.05, 0.22, 0.046, 0]
        ntlab.gripperSetPose(gripper_pos)
        ntlab.gripperExecute()

        input("Press 'Enter' to next.") # grip
        position = [
            0.00252282,
            0.149731,
            0.33767,
            0.0123818,
            -0.746716,
            0.665024,
            0.00213275,
        ]
        ntlab.cobottaExecutePoseGoal(position)

        input("Press 'Enter' to next.")
        ntlab.gripperGripLimitTorque()

        input("Press 'Enter' to next.") # rotate
        position = [
            -0.00295548,
            0.188509,
            0.401929,
            0.0123125,
            -0.746716,
            0.665025,
            0.00229287,
        ]
        ntlab.cobottaExecutePoseGoal(position)

        input("Press 'Enter' to next.") # current pos + rotate todo
        gripper_pos = [0.235, -0.05, 0.235, 0.05, 0]
        ntlab.gripperSetPose(gripper_pos)
        ntlab.gripperExecute()

        input("Press 'Enter' to next.") # place
        position = [
            0.00475795,
            0.260686,
            0.339046,
            0.0121942,
            -0.746748,
            0.664991,
            0.00225271,
        ]
        ntlab.cobottaExecutePoseGoal(position)

        input("Press 'Enter' to next.") # current pos + open slightly + finger up todo
        gripper_pos = [0.235, -0.05, 0.235, 0.05, 0]
        ntlab.gripperSetPose(gripper_pos)
        ntlab.gripperExecute()

        input("Press 'Enter' to start.") # standby
        ntlab = GripperNTLab()
        position = [
            0.00206299,
            0.148964,
            0.372349,
            0.0123763,
            -0.74677,
            0.664963,
            0.00219526,
        ]
        ntlab.cobottaExecutePoseGoal(position)

        input("Press 'Enter' to next.") # standby
        gripper_pos = [0.19, -0.06, 0.19, 0.06, 0]
        ntlab.gripperSetPose(gripper_pos)
        ntlab.gripperExecute()

        input("Press 'Enter' to end.")

    except rospy.ROSInterruptException:
        print("error")
        return

if __name__ == "__main__":
    main()
