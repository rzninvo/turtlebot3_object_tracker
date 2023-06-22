#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.yolo.utils.plotting import Annotator
from ultralytics.yolo.engine.results import Results
from turtlebot3_object_tracker.srv import DetectionData, DetectionDataResponse

# ROS
import rospy
from sensor_msgs.msg import Image


class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Instantiate your YOLO object detector/classifier model
        self.model: YOLO = YOLO('/home/rohamzn/Robotics-Course/catkin_ws/src/turtlebot3_object_tracker/yolo/yolov8n.pt')

        # TODO: Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber = rospy.Subscriber('/follower/camera/image', Image, callback=self.camera_listener)

        # TODO: You need to update results each time you call your model
        self.results: Results = self.model(self.image_np)

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('human_detection_server', DetectionData, self.get_detection_data)

        self.update_view()


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)


    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue


                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                # Predicting results
                self.results = self.model(self.image_np, verbose = False)
                res_plotted = self.results[0].plot()
                #frame = copy.deepcopy(self.image_np)
                # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                # self.annotator : Annotator = Annotator(self.image_np)
                # self.annotator.box_label(self.results[0].boxes[0], label= 'Person')
                # result_image = self.annotator.result()

                cv2.imshow("robot_view", cv2.cvtColor(res_plotted, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

        except rospy.exceptions.ROSInterruptException:
            pass

    def get_detection_data(self, req):
        result = self.results[0]

        box_data = {}

        response = DetectionDataResponse()

        for box in result.boxes:
            coordinates = box.xyxy[0].tolist()
            bb_x1, bb_y1, bb_x2, bb_y2 = [round(x) for x in coordinates]
            bb_cx = (bb_x2 + bb_x1)/2
            bb_cy = (bb_y1 + bb_y2)/2
            bb_width = bb_x2 - bb_x1
            bb_height = bb_y2 - bb_y1
            class_id = box.cls[0].item()
            box_data[result.names[class_id]] = [bb_cx, bb_cy, bb_width, bb_height]

        if req.label in box_data.keys():
            response.bb_cx = box_data[req.label][0]
            response.bb_cy = box_data[req.label][1]
            response.bb_width = box_data[req.label][2]
            response.bb_height = box_data[req.label][3]
            response.img_width = self.image_res[1]
            response.img_height = self.image_res[0]
            response.flag = 'found'
        else:
            response.bb_cx = self.image_res[1] / 2
            response.bb_cy = self.image_res[0] / 2
            response.bb_width = 0
            response.bb_height = 0
            response.img_width = self.image_res[1]
            response.img_height = self.image_res[0]
            response.flag = 'not_found'

        return response

if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()