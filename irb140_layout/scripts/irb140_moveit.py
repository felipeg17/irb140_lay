# General imports
import sys
import copy
import time

from sympy import false
import rospy

# ROS and Moveit
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
from std_msgs.msg import Float64

# Transf  and RTB
from tf.transformations import *
from spatialmath import *
from spatialmath.base import *
import roboticstoolbox as rtb

# Tools
import numpy as np

#############################################################################
# Starts the moveit_commander node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python", anonymous=False)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

# Sets the movegroup
# The movegroup is the moveit object for planning, it constains the mechanism, joitns, etc.
# The commander allows the move_group motion
group_name = "irb140_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Planner params
move_group.set_planner_id("PRMstarkConfigDefault") # Planner PRM, try RRT e.g.
move_group.set_planning_time(5)
move_group.set_max_velocity_scaling_factor(0.5)
move_group.set_max_acceleration_scaling_factor(0.5)
#############################################################################


def joint_motion(q):
    # Joint motion 
    # Get the current joints
    joint_goal = move_group.get_current_joint_values()
    # Set new values
    joint_goal[0] = q[0]
    joint_goal[1] = q[1]
    joint_goal[2] = q[2]
    joint_goal[3] = q[3]
    joint_goal[4] = q[4]
    joint_goal[5] = q[5]
    # Start the motion
    move_group.go(joint_goal, wait=True)
    # Finish the motion
    move_group.stop()

def  ws_motion(pose):
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose[0]
    pose_goal.position.y = pose[1]
    pose_goal.position.z = pose[2]
    pose_goal.orientation.x = pose[3]
    pose_goal.orientation.y = pose[4]
    pose_goal.orientation.z = pose[5]
    pose_goal.orientation.w = pose[6]
    # Query the motion
    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    # Stops the motion
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == "__main__":
    # Set clean prints
    np.set_printoptions(suppress=True)

    gripper_topic ='/gripper_controller/command'
    is_gripper_topic = False
    for topic in rospy.get_published_topics():
        if '/gripper_controller/command' in topic: is_gripper_topic = True
    
    # if is_gripper_topic:
    print('gripper detected')
    gripper_pub = rospy.Publisher('/gripper_controller/command', Float64, queue_size=5)
    gripper_pub.publish(5)
    print('gripper_open')
    rospy.sleep(1)

    joint_motion(np.array([0, 0, 0, 0, np.pi/6, 0]))
    # time.sleep(0.1)
    # joint_motion(np.array([0.694, 0.289, -0.469, 0, 1.75, 0.695]))
    # [0.5067793457656542, 0.19498238658623812, -0.3528302904797309, 0.0004234713342130547, 1.729444616118922, 0.5077151482134368]
    joint_motion(np.array([0.50678, 0.195, -0.35283, 0, 1.72944, 0.507715]))
    time.sleep(0.1)
    # Shows the joints values and the pose
    print(move_group.get_current_joint_values())
    print(move_group.get_current_pose().pose)
    #############################################################################

    # Get the current pose
    current_pose = moveit_commander.conversions.pose_to_list(move_group.get_current_pose().pose)
    # Conversion to a ROS msg
    R = troty(np.pi)
    qua_goal = quaternion_from_matrix(R)
    pregrasp_pose = numpy.concatenate([np.array([0.45, 0.25, 0.45]),qua_goal])
    corner_one = numpy.concatenate([np.array([0.3, 0, 0.37]),qua_goal])
    corner_two = numpy.concatenate([np.array([0.3, 0.5, 0.37]),qua_goal])
    # corner_three = numpy.concatenate([np.array([0.6, 0.6, 0.37]),qua_goal])
    corner_four = numpy.concatenate([np.array([0.6, 0, 0.37]),qua_goal])
    # ws_motion(pregrasp_pose)
    # print(move_group.get_current_joint_values())
    R2 = R.dot(trotz(-8.87,'deg'))
    qua_grasp = quaternion_from_matrix(R2)
    ws_motion(numpy.concatenate([np.array([0.45, 0.08, 0.42]),qua_grasp]))
    gripper_pub.publish(5)
    rospy.sleep(0.5)
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position.z -= 0.058  # First down up (z)
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    print(fraction)
    move_group.execute(plan, wait=True)
    # ws_motion(numpy.concatenate([np.array([0.58, 0.216, 0.37]),qua_grasp]))
    rospy.sleep(0.5)
    gripper_pub.publish(-.5)
    # rospy.sleep(0.5)
    # gripper_pub.publish(-2)
    # Shows the joints values and the pose
    # print(move_group.get_current_joint_values())
    # print(move_group.get_current_pose().pose)
    ############################################################################

    #############################################################################
    # Cartesian Motion
    # waypoints = []
    # wpose = move_group.get_current_pose().pose
    # n = 10
    # x_inter = np.linspace(wpose.position.x,corner_one[0],n)
    # y_inter = np.linspace(wpose.position.y,corner_one[1],n)
    # z_inter = np.linspace(wpose.position.z,corner_one[2],n)
    # for i in range(n):
        # wpose.position.x = x_inter[i]
        # wpose.position.y = y_inter[i]
        # wpose.position.z = z_inter[i]
        # waypoints.append(copy.deepcopy(wpose))
    # scale = 1
    # wpose.position.z -= scale * 0.05  # First down up (z)
    # waypoints.append(copy.deepcopy(wpose))
    # wpose.position.x -= scale * 0.05  # First down up (z)
    # waypoints.append(copy.deepcopy(wpose))
    # (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # print(fraction)
    # Execute motion
    # move_group.execute(plan, wait=True)