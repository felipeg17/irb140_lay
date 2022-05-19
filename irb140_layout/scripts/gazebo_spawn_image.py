import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
from spatialmath.base import *
import roboticstoolbox as rtb
from tf.transformations import *

class GetRosImage:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_received = False
        # Connect image topic
        # rgb_topic = "/camera/color/image_raw"
        rgb_topic = "/cam/color/image_raw"
        self.image_sub = rospy.Subscriber(rgb_topic, Image, self.callback_rgb)
        # depth_topic = "/camera/depth/image_rect_raw"
        depth_topic = 'cam/depth/image_raw'
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.callback_depth)
        # Allow up to one second to connection
        rospy.sleep(0.5)

    def callback_rgb(self, data):
        # Convert image to OpenCV format
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.image_received = True
        self.rgb_image = rgb_image
    
    def callback_depth(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)
        self.depth_image = depth_image

    def take_picture(self, names, save=True):
        if self.image_received:
            if save == True:
                cv2.imwrite(names[0], self.rgb_image)
                cv2.imwrite(names[1],cv2.normalize(self.depth_image,self.depth_image,0,255,cv2.NORM_MINMAX))
                cv2.imwrite(names[2],cv2.normalize(self.depth_image,self.depth_image,0,255,cv2.NORM_MINMAX))
            return (self.rgb_image, self.depth_image)
        else:
            return False

def spawn_sdf(name,path,pose):
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name = name,
        model_xml = open(path, 'r').read(),
        initial_pose = pose,
        reference_frame = 'world')
    return None

def delete_model(name):
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model_client(model_name=name)
    return None

if __name__ == "__main__":
    objects = ['banana', 'foam', 'tuna', 'tazon', 'apple', 'c_cup', 'sugar']
    # paths = ['/home/felipe/catkin_ws/src/irb140_lay/irb140_layout/objects/banana/model.sdf',
    #             '/home/felipe/catkin_ws/src/irb140_lay/irb140_layout/objects/foam/model.sdf',
    #             '/home/felipe/catkin_ws/src/irb140_lay/irb140_layout/objects/tuna/model.sdf']
    # positions = [[0.4, 0.1, 0.3], 
    #              [0.45, 0.25, 0.3],         
    #              [0.5, 0.4, 0.3]]
    paths = ['/home/felipe/catkin_ws/src/irb140_sim/objetos/banana/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/foam/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/tuna/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/tazon/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/apple/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/c_cups/model.sdf',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/sugar/model.sdf']

    rospy.init_node('insert_object',log_level=rospy.INFO)
    init_pose=Pose()
    camera = GetRosImage()
    

    for reps in range(10):
        # init_pose.position.x = 0.55-0.1*np.random.rand()
        # init_pose.position.y = 0.05-0.1*np.random.rand()
        init_pose.position.x = 0.45
        init_pose.position.y = 0.25
        init_pose.position.z = 0
        R = trotz(2-4*np.random.rand(),'deg')
        # R = trotz(0,'deg')
        qua_goal = quaternion_from_matrix(R)
        init_pose.orientation.x = qua_goal[0]
        init_pose.orientation.y = qua_goal[1]
        init_pose.orientation.z = qua_goal[2]
        init_pose.orientation.w = qua_goal[3]
        spawn_sdf('bin',
                '/home/felipe/catkin_ws/src/irb140_sim/objetos/bin/model.sdf',
                init_pose)
        rospy.sleep(0.5)
        for i in range(len(objects)):        
            # init_pose.position.x = 0.7-0.4*np.random.rand()+0.05
            # init_pose.position.y = 0.25-0.5*np.random.rand()+0.35
            init_pose.position.x = 0.65-0.3*np.random.rand()
            init_pose.position.y = 0.3-0.6*np.random.rand()+0.2
            init_pose.position.z = 0.01
            R = trotz(np.pi-2*np.pi*np.random.rand())
            qua_goal = quaternion_from_matrix(R)
            init_pose.orientation.x = qua_goal[0]
            init_pose.orientation.y = qua_goal[1]
            init_pose.orientation.z = qua_goal[2]
            init_pose.orientation.w = qua_goal[3]
            spawn_sdf(objects[i],paths[i],init_pose)
            print(init_pose)

        rospy.sleep(1)
        pic_names = ['rgb'+str(reps+1)+'.jpg', 'depth'+str(reps+1)+'.jpg', 'depth'+str(reps+1)+'.tiff']
        camera_result = camera.take_picture(pic_names)
        if isinstance(camera_result,tuple):
            rgb_img, depth_img = camera_result
            print("Saved image")
        else:
            print("No images received")
        

        for i in range(len(objects)):   
            delete_model(objects[i])
        # rospy.sleep(0.2)
        delete_model('bin')
        rospy.sleep(0.2)
    
