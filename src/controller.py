#!/usr/bin/python3

# ROS
import rospy
from geometry_msgs.msg import Twist
from turtlebot3_object_tracker.srv import DetectionData, DetectionDataResponse
import math


class Controller:
    def __init__(self) -> None:
        # Use these Twists to control your robot
        self.move = Twist()
        self.move.linear.x = 0.1
        self.freeze = Twist()


        # Bounding-box data
        self.bb_cx = 0
        self.bb_cy = 0
        self.bb_width = 0
        self.bb_height = 0
        self.img_width = 0
        self.img_height = 0
        
        # The "p" parameter for your p-controller, TODO: you need to tune this
        self.angular_vel_coef = 0.005

        # TODO: Create a service proxy for your human detection service
        self.get_bb_from_server('person')

        
        # TODO: Create a publisher for your robot "cmd_vel"
        self.cmd_vel = rospy.Publisher('/follower/cmd_vel', Twist, queue_size=5)

    def get_bb_from_server(self, label):
        rospy.wait_for_service('human_detection_server')
        try:
            gbb_data_service = rospy.ServiceProxy('human_detection_server', DetectionData)
            response : DetectionDataResponse = gbb_data_service(label)
            self.bb_cx = response.bb_cx
            self.bb_cy = response.bb_cy
            self.bb_width = response.bb_width
            self.bb_height = response.bb_height
            self.img_width = response.img_width
            self.img_height = response.img_height
            self.flag = response.flag
            
        except rospy.ServiceException as e:
           rospy.loginfo(f'ERROR from human_detection_server service: {e}')

    def calculate_rotation_angle(self):
        img_center_x = self.img_width / 2
        img_center_y = self.img_height / 2
        return img_center_x - self.bb_cx

    def run(self) -> None:
        try:
            while not rospy.is_shutdown():
                # TODO: Call your service, ride your robot
                self.get_bb_from_server('person')
                err = self.calculate_rotation_angle()
                angular_P = self.angular_vel_coef * err
                if self.flag == 'found':
                    self.move.angular.z = angular_P
                    self.cmd_vel.publish(self.move)
                pass

        except rospy.exceptions.ROSInterruptException:
            pass
                

if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    
    controller = Controller()
    controller.run()
    

