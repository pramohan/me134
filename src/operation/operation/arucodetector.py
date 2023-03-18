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
from math import atan2, cos, sin, sqrt, pi

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
        self.mode = "in"

        self.pink_color = [155, 158, 151]
        self.green_color = [89, 100, 121]

        # Create subscriber for the image from the usb_camera.
        self.sub = self.create_subscription(Image, 'usb_cam/image_raw', self.process, 1)
        self.img_pub = self.create_publisher(Image, name+'/image_raw', 3)

        self.pub_M = self.create_publisher(Float32MultiArray, '/matches', 1)
        self.pub_NM = self.create_publisher(Float32MultiArray, '/no_match', 1)
        self.pub_rem = self.create_publisher(Float32MultiArray, '/on_blue', 1)


    # Create a temporary handler to grab the info.
    def cb(self, msg):
            self.camD = np.array(msg.d).reshape(5)
            self.camK = np.array(msg.k).reshape((3,3))
            self.camw = msg.width
            self.camh = msg.height
            self.caminfoready = True


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


    # Color correction
    def cc(self, color):
        return max(0, min(255, color))


    # Get the contours given a color and interval ranges
    def get_contours(self, hsv, color, h_range, s_range, v_range, iter, min_area = None):
        hsvLower = (self.cc(color[0]-h_range), self.cc(color[1]-s_range), self.cc(color[2]-v_range))
        hsvUpper = (self.cc(color[0]+h_range), self.cc(color[1]+s_range), self.cc(color[2]+v_range))

        binary = cv2.inRange(hsv, hsvLower, hsvUpper)
        binary = cv2.erode( binary, None, iterations=iter)
        binary = cv2.dilate(binary, None, iterations=iter)

        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if min_area is not None:
            contours = tuple([x for x in contours if cv2.contourArea(x) > min_area])

        return contours


    # Get the orientation of an object
    def get_orientation(self, contour):
        ratio = self.get_ratio(contour)

        if abs(ratio - 1) > 0.6:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect) 
            box = np.int0(box)
            height = rect[1][0]
            width = rect[1][1]
            tl_corner = box[0]
            tr_corner = box[3]
            bl_corner = box[1]
            if height > width:
                angle = atan2(tl_corner[0] - tr_corner[0], tl_corner[1] - tr_corner[1])
            else:
                angle = atan2(bl_corner[0] - tl_corner[0], bl_corner[1] - tl_corner[1])
        else:
            hull = cv2.convexHull(contour)
            cx, cy = self.find_center(hull)
            def dist_to_center(point):
                point = point[0]
                return (cx - point[0]) ** 2 + (cy - point[1]) ** 2
            max_point = max(hull, key = dist_to_center)[0]
            angle = atan2(cx - max_point[0], cy - max_point[1])

        return angle


    # Get the contours of the borders
    def set_hole_contours(self, hsv):
        contours = self.get_contours(hsv, self.green_color, 10, 70, 100, 2)
        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                self.hole_contours.append(contour)


    def should_match(self, piece, sim):
        ratio = self.get_ratio(piece)

        if abs(ratio - 1) > 0.6:
            return sim < 0.4
        else:
            return sim < 0.12


    def get_ratio(self, piece):
        rect = cv2.minAreaRect(piece)

        height = rect[1][0]
        width = rect[1][1]

        if height >= width:
            ratio = height / width
        else:
            ratio = width / height

        return ratio

    def process(self, msg):
        # # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

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
                    self.M = cv2.getPerspectiveTransform(undistort_coords.reshape((-1, 1, 2)), np.array(table_coords, np.float32).reshape((-1, 1, 2)))
                    self.get_logger().info("DONE SETTING UP CAMERAS")

            # Get the contours of the borders
            self.set_hole_contours(hsv)

        elif not self.aruco_loc_flag: # CODE FOR TAKING PIECES OUT
            piece_loc_arr = []

            blue_color = [109, 175, 177]
            contours = self.get_contours(hsv, blue_color, 5, 60, 100, 3)

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

                piece_contours = self.get_contours(hsv, self.pink_color, 10, 100, 100, 1)
                if len(piece_contours) > 0:
                    for piece_contour in piece_contours:
                        if cv2.contourArea(piece_contour) > 1000:
                            cx, cy = self.find_center(piece_contour)
                            if (low_x < cx < high_x) and (low_y < cy < high_y):
                                points = self.get_transform(cx, cy)
                                piece_loc_arr.append(points[0])
                                piece_loc_arr.append(points[1])

                #self.get_logger().info(str(piece_loc_arr))
                self.pub_rem.publish(Float32MultiArray(data = np.array(piece_loc_arr)))
            
            goto_array = []
            nonmatching = []

            contours = self.get_contours(hsv, self.pink_color, 10, 100, 100, 1)

            piece_list = []

            if len(contours) > 0:
                for piece in contours:
                    if cv2.contourArea(piece) > 1000:
                        piece_list.append(piece)

            holes = self.get_contours(hsv, self.green_color, 10, 70, 100, 2, min_area = 2900)
            # self.get_logger().info(str(len(holes)))

            # match_dict = {}

            if len(holes) > 0 and len(piece_list) > 0:
                for piece in piece_list:
                    best_coeff = 10
                    best_hole = None
                    for hole in holes:
                        sim_coeff = cv2.matchShapes(piece, hole, 1, 0.0)
                        if sim_coeff < best_coeff:
                            best_coeff = sim_coeff
                            best_hole = hole

                    if self.should_match(piece, best_coeff):
                        # match_dict[str(best_coeff)] = (piece, best_hole) 
                        piece_x, piece_y = self.find_center(piece)
                        piece_location = self.get_transform(piece_x, piece_y)
                        piece_orientation = self.get_orientation(piece) 

                        hole_x, hole_y = self.find_center(best_hole)
                        hole_location = self.get_transform(hole_x, hole_y)
                        hole_orientation = self.get_orientation(best_hole)

                        angle_diff = int(np.rad2deg(hole_orientation - piece_orientation))

                        goto_array.extend([piece_location[0], piece_location[1], hole_location[0], hole_location[1], angle_diff])
                    else:
                        piece_x, piece_y = self.find_center(piece)
                        piece_location = self.get_transform(piece_x, piece_y)
                        nonmatching.extend([piece_location[0], piece_location[1]])

            # if len(match_dict.keys()) != 0:
            #     dict_keys = list(match_dict.keys())
            #     dict_keys.sort()
            #     for key in dict_keys:
            #         piece, best_hole = match_dict[key]
            #         piece_x, piece_y = self.find_center(piece)
            #         piece_location = self.get_transform(piece_x, piece_y)
            #         piece_orientation = self.get_orientation(piece) 

            #         hole_x, hole_y = self.find_center(best_hole)
            #         hole_location = self.get_transform(hole_x, hole_y)
            #         hole_orientation = self.get_orientation(best_hole)

            #         angle_diff = int(np.rad2deg(hole_orientation - piece_orientation))

            #         goto_array.extend([piece_location[0], piece_location[1], hole_location[0], hole_location[1], angle_diff])

            #self.get_logger().info(str(goto_array))
            self.pub_M.publish(Float32MultiArray(data = np.array(goto_array))) 
            self.pub_NM.publish(Float32MultiArray(data = np.array(nonmatching)))              
                        

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
