#!/usr/bin/env python3
#
#   arucodetector.py
#
#   Node to do aruco stuff
#

import numpy as np
import rclpy
import cv2
import cv_bridge
import time

from rclpy.node             import Node
from sensor_msgs.msg        import CameraInfo
from sensor_msgs.msg        import Image
from std_msgs.msg           import Float32MultiArray

if __name__ == "__main__":
    # Prepare the print format.
    np.set_printoptions(precision=6, suppress=True)

class ArucoDetector(Node):
    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info("Waiting for camera info...")
        sub = self.create_subscription(CameraInfo, '/usb_cam/camera_info', self.cb, 1)
        self.caminfoready = False
        while not self.caminfoready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        self.bridge = cv_bridge.CvBridge()
        self.dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()

        self.aruco_loc_flag = True
        self.aruco_locs = []
        self.aruco_table_locs = {"0":[-0.005, 0.345], "1":[0, -0.331], "4": [-0.79, -0.315], "5":[-0.71, 0.345]}

        self.hole_contours = []

        # Create subscriber for the image from the usb_camera.
        self.sub = self.create_subscription(Image, 'usb_cam/image_raw', self.process, 1)
        self.img_pub = self.create_publisher(Image, name+'/image_raw', 3)

        self.pub = self.create_publisher(Float32MultiArray, '/aruco_location', 1)


    # Create a temporary handler to grab the info.
    def cb(self, msg):
            self.camD = np.array(msg.d).reshape(5)
            self.camK = np.array(msg.k).reshape((3,3))
            self.camw = msg.width
            self.camh = msg.height
            self.caminfoready = True

    # def get_aruco_pos(self):
    #     points = []
    #     (boxes, ids, rejected) = cv2.aruco.detectMarkers(
    #         frame, self.dict, parameters=self.params)
    #     for (box, id) in zip(boxes, ids.flatten()):
    #         desired_aruco = 3
    #         if id == desired_aruco:
    #             (topLeft, topRight, bottomRight, bottomLeft) = box[0]
    #             center = (topLeft + bottomRight)/2
    #             image_loc = np.array([center], np.float32).reshape((-1, 1, 2))
    #             points = cv2.perspectiveTransform(image_loc, self.M)
    #             points = list(np.squeeze(points))
    #             points.append(0.05)
    #             points = np.array(points)
    #     return points

    # Find the center of mass of a contour
    def find_center(self, contour):
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return cx, cy


    # Transform pixel coordinates into world coordinates
    def get_transform(self, cx, cy):
        piece_loc = np.array([[cx, cy]], np.float32).reshape((-1, 1, 2))
        points = cv2.perspectiveTransform(piece_loc, self.M)
        points = list(np.squeeze(points))
        return points


    # Get the contours of the borders
    def set_hole_contours(self, hsv):
        def color_correction(color):
                return max(0, min(255, color))

        color = [89, 100, 121] # HOLE COLORS
        hsvLower = (color_correction(color[0]-10), color_correction(color[1]-60), color_correction(color[2]-100))
        hsvUpper = (color_correction(color[0]+10), color_correction(color[1]+60), color_correction(color[2]+100))
        binary = cv2.inRange(hsv, hsvLower, hsvUpper)
        binary = cv2.erode( binary, None, iterations=2)
        self.binary = cv2.dilate(binary, None, iterations=2)
        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                self.hole_contours.append(contour)


    def process(self, msg):
        # # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        def color_correction(color):
                return max(0, min(255, color))

        # # Detect.
        if self.aruco_loc_flag:
            (boxes, ids, rejected) = cv2.aruco.detectMarkers(
                frame, self.dict, parameters=self.params)

            table_coords = []
            # Loop over each marker: the list of corners and ID for this marker.
            if len(boxes) >= 4:
                for (box, id) in zip(boxes, ids.flatten()):
                    (topLeft, topRight, bottomRight, bottomLeft) = box[0]
                    center = (topLeft + bottomRight)/2
                    if str(id) in list(self.aruco_table_locs.keys()):
                        self.aruco_locs.append(center)
                        table_coords.append(self.aruco_table_locs[str(id)])

                if len(table_coords) == 4 and len(self.aruco_locs) == 4:
                    self.aruco_loc_flag = False
                    self.aruco_locs = np.array(self.aruco_locs, np.float32).reshape((-1, 1, 2))
                    undistort_coords = cv2.undistortPointsIter(self.aruco_locs, self.camK, self.camD, None, self.camK, (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 0.03))
                    # self.get_logger().info(str(len(undistort_coords)))
                    # self.get_logger().info(str(len(self.aruco_locs)))
                    self.M = cv2.getPerspectiveTransform(undistort_coords.reshape((-1, 1, 2)), np.array(table_coords, np.float32).reshape((-1, 1, 2)))
                    self.get_logger().info("DONE SETTING UP CAMERAS")

            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            # Get the contours of the borders
            self.set_hole_contours(hsv)

        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            # Bone
            piece_loc_arr = []

            color = [109, 175, 177]
            hsvLower = (color_correction(color[0]-5), color_correction(color[1]-60), color_correction(color[2]-150))
            hsvUpper = (color_correction(color[0]+5), color_correction(color[1]+60), color_correction(color[2]+150))

            hsvLower = (104, 175-60, 27)
            hsvUpper = (114, 175+60, 255)
            binary = cv2.inRange(hsv, hsvLower, hsvUpper)
            binary = cv2.erode( binary, None, iterations=3)
            binary = cv2.dilate(binary, None, iterations=3)

            (contours, hierarchy) = cv2.findContours(
                binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
            # Pick the largest contour.
                contour = max(contours, key=cv2.contourArea)

                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                low_x = min([x[0] for x in box])
                high_x = max([x[0] for x in box])
                low_y = min([y[1] for y in box])
                high_y = max([y[1] for y in box])

                hsvLower = (150, 158-70, 51)
                hsvUpper = (160, 158+70, 251)
                binary = cv2.inRange(hsv, hsvLower, hsvUpper)      
                binary = cv2.erode(binary, None, iterations=1)
                binary = cv2.dilate(binary, None, iterations=1)
                (piece_contours, hierarchy) = cv2.findContours(
                    binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(piece_contours) > 0:
                    for piece_contour in piece_contours:
                        if cv2.contourArea(piece_contour) > 1000:
                            M = cv2.moments(piece_contour)
                            cx = int(M['m10']/M['m00'])
                            cy = int(M['m01']/M['m00'])
                            if (low_x < cx < high_x) and (low_y < cy < high_y):
                                points = self.get_transform(cx, cy)
                                piece_loc_arr.append(points[0])
                                piece_loc_arr.append(points[1])

                # self.get_logger().info(str(len(piece_loc_arr)))
                self.pub.publish(Float32MultiArray(data = np.array(piece_loc_arr)))

        for hole in self.hole_contours:
            cv2.drawContours(frame, hole, -1, (  0,   0, 255), 2)
            # coeff = cv2.matchShapes(contour, hole, 1, 0.0)
            # label = str(coeff)
            # cx, cy = self.find_center(hole)
            # textbox = cv2.rectangle(frame, (cx, cy-25), (cx + 250, cy + 10), (255,255,255), -1)
            # cv2.putText(frame, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
            


def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = ArucoDetector('demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
