from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import numpy as np


def spawn_sdf(name,path,pose):
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name = name,
        model_xml = open(path, 'r').read(),
        initial_pose = pose,
        reference_frame = 'world')
    return None

if __name__ == "__main__":
    objects = ['banana', 'foam', 'tuna']
    paths = ['/home/felipe/catkin_ws/src/irb140_lay/irb140_layout/objects/banana/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_lay/irb140_layout/objects/foam/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_lay/irb140_layout/objects/tuna/model.sdf']
    positions = [[0.4, 0.07, 0.3], [0.4, 0.3, 0.3], [0.55, 0.25, 0.3]]
    rospy.init_node('insert_object',log_level=rospy.INFO)
    init_pose=Pose()
    for i in range(len(objects)):        
        init_pose.position.x = positions[i][0]+(0.1-0.2*np.random.rand())
        init_pose.position.y = positions[i][1]+(0.1-0.2*np.random.rand())
        init_pose.position.z = positions[i][2]
        init_pose.orientation.x = 0
        init_pose.orientation.y = 0
        init_pose.orientation.z = 0
        init_pose.orientation.w = 0
        spawn_sdf(objects[i],paths[i],init_pose)