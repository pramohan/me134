#!/usr/bin/env python3
#
#   balldetector.py
#
#   Detect the tennis balls with OpenCV.
#
#   Node:           /balldetector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /balldetector/image_raw     Debug image
#
import cv2
import numpy as np
from math import atan2, cos, sin, sqrt, pi

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image


#
#  Detector Node Class
#
class DetectorNode(Node):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        self.board = np.array([])

        # Report.
        self.get_logger().info("Ball detector running...")

    def drawAxis(self, img, p_, q_, color, scale):
        p = list(p_)
        q = list(q_)
        
        ## [visualization1]
        angle = atan2(p[1] - q[1], p[0] - q[0]) # angle in radians
        hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
        
        # Here we lengthen the arrow by a factor of scale
        q[0] = p[0] - scale * hypotenuse * cos(angle)
        q[1] = p[1] - scale * hypotenuse * sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        
        # create the arrow hooks
        p[0] = q[0] + 9 * cos(angle + pi / 4)
        p[1] = q[1] + 9 * sin(angle + pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        
        p[0] = q[0] + 9 * cos(angle - pi / 4)
        p[1] = q[1] + 9 * sin(angle - pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv2.LINE_AA)
        ## [visualization1]

    def getOrientation2(self, pts, img):
        hull = cv2.convexHull(pts)
        M = cv2.moments(hull)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cntr = [cx, cy]
        def dist_to_center(point):
            point = point[0]
            return (cx - point[0]) ** 2 + (cy - point[1]) ** 2
        max_point = max(hull, key = dist_to_center)[0]
        cv2.line(img, (cx, cy), (max_point[0], max_point[1]), (255, 255, 0), 3, cv2.LINE_AA)
        angle = atan2(cx - max_point[0], cy - max_point[1])
        label = str(int(np.rad2deg(angle))) + " degrees"
        textbox = cv2.rectangle(img, (cntr[0] + 30, cntr[1]-25), (cntr[0] + 280, cntr[1] + 10), (255,255,255), -1)
        cv2.putText(img, label, (cntr[0] + 30, cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        return angle

    def getOrientation(self, pts, img):
        # sz = len(pts)
        # data_pts = np.empty((sz, 2), dtype=np.float64)
        # for i in range(data_pts.shape[0]):
        #     data_pts[i,0] = pts[i,0,0]
        #     data_pts[i,1] = pts[i,0,1]
        # mean = np.empty((0))
        # mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        # cntr = (int(mean[0,0]), int(mean[0,1]))
        # angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
        # label = str(int(np.rad2deg(angle))) + " degrees"
        # textbox = cv2.rectangle(img, (cntr[0] + 30, cntr[1]-25), (cntr[0] + 280, cntr[1] + 10), (255,255,255), -1)
        # cv2.putText(img, label, (cntr[0] + 30, cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        # return angle
        rect = cv2.minAreaRect(pts)
        box = cv2.boxPoints(rect) 
        box = np.int0(box)
        height = rect[1][0]
        width = rect[1][1]
        tl_corner = box[0]
        tr_corner = box[3]
        bl_corner = box[1]
        if height > width:
            cv2.line(img, (tl_corner[0], tl_corner[1]), (tr_corner[0], tr_corner[1]), (255, 255, 0), 3, cv2.LINE_AA)
            angle = atan2(tl_corner[0] - tr_corner[0], tl_corner[1] - tr_corner[1])
        else:
            cv2.line(img, (tl_corner[0], tl_corner[1]), (bl_corner[0], bl_corner[1]), (255, 255, 0), 3, cv2.LINE_AA)
            angle = atan2(bl_corner[0] - tl_corner[0], bl_corner[1] - tl_corner[1])

        M = cv2.moments(pts)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cntr = [cx, cy]
        # def dist_to_center(point):
        #     point = point[0]
        #     return (cx - point[0]) ** 2 + (cy - point[1]) ** 2
        # max_point = min(pts, key = dist_to_center)[0]
        # # cv2.line(img, (cx, cy), (max_point[0], max_point[1]), (255, 255, 0), 3, cv2.LINE_AA)
        # angle = atan2(cx - max_point[0], cy - max_point[1])
        label = str(int(np.rad2deg(angle))) + " degrees"
        # label = str(angle) + " degrees"
        textbox = cv2.rectangle(img, (cntr[0] + 30, cntr[1]-25), (cntr[0] + 280, cntr[1] + 10), (255,255,255), -1)
        cv2.putText(img, label, (cntr[0] + 30, cntr[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        return angle

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    def cc(self, color):
        return max(0, min(255, color))

    def get_contours(self, hsv, color, h_range, s_range, v_range, iter):
        hsvLower = (self.cc(color[0]-h_range), self.cc(color[1]-s_range), self.cc(color[2]-v_range))
        hsvUpper = (self.cc(color[0]+h_range), self.cc(color[1]+s_range), self.cc(color[2]+v_range))

        binary = cv2.inRange(hsv, hsvLower, hsvUpper)
        binary = cv2.erode( binary, None, iterations=iter)
        binary = cv2.dilate(binary, None, iterations=iter)

        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.board = frame

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Help to determine the HSV range...
        if True:
            # Draw the center lines.  Note the row is the first dimension.
            (H, W, _) = frame.shape
            xc = W//2
            yc = H//2
            frame = cv2.line(frame, (xc,0), (xc,H-1), self.white, 1)
            frame = cv2.line(frame, (0,yc), (W-1,yc), self.white, 1)

            # Report the center HSV values.  Note the row comes first.
            self.get_logger().info(
                "HSV = (%3d, %3d, %3d)" % tuple(hsv[yc, xc]))

        # Get piece contour
        # color = [156, 158, 151]
        # hsvLower = (color_correction(color[0]-10), color_correction(color[1]-60), color_correction(color[2]-100))
        # hsvUpper = (color_correction(color[0]+10), color_correction(color[1]+60), color_correction(color[2]+100))
        # binary = cv2.inRange(hsv, hsvLower, hsvUpper)
        # binary = cv2.erode( binary, None, iterations=2)
        # binary = cv2.dilate(binary, None, iterations=2)
        # (contours, hierarchy) = cv2.findContours(
        #     binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # if len(contours) != 0:
        #     pink_piece_contour = max(contours, key=cv2.contourArea)
        #     cv2.drawContours(frame, pink_piece_contour, -1, self.blue, 2)
        
        # Threshold in Hmin/max, Smin/max, Vmin/max
        color = [92, 100, 121] # HOLE COLORS
        # color = [155, 158, 151] # PINK PIECE
        # color = [109, 175, 177] # BLUE COLOR

        # saturation: 70 for holes, 100 for red
        contours = self.get_contours(hsv, color, 10, 70, 100, 2) # HOLES
        # contours = self.get_contours(hsv, color, 10, 100, 100, 1) # PIECES
        # contours = self.get_contours(hsv, color, 5, 60, 100, 3)

        # Draw all contours on the original image for debugging.
        cv2.drawContours(frame, contours, -1, self.blue, 2)

        # Only proceed if at least one contour was found.  You may
        # also want to loop over the contours...
        if len(contours) > 0:
            # Pick the largest contour.
            for contour in contours:
                if cv2.contourArea(contour) > 3000:
                    self.get_logger().info(str(cv2.contourArea(contour)))
                #     rect = cv2.minAreaRect(contour)
                #     box = cv2.boxPoints(rect)
                #     box = np.int0(box)
                #     # cv2.drawContours(frame,[box],0,(0,0,255),2)
                #     height = rect[1][0]
                #     width = rect[1][1]
                #     if height >= width:
                #         ratio = height / width
                #     else:
                #         ratio = width / height
                #     if abs(ratio - 1) > 0.6:
                #         self.getOrientation(contour, frame)
                #     else:
                #         self.getOrientation2(contour, frame)

                    pieces = self.get_contours(hsv, [155, 158, 151], 10, 100, 100, 1)

                    if len(pieces) > 0:
                        for piece in pieces:
                            if cv2.contourArea(piece) > 1000:
                                cv2.drawContours(frame, piece, -1, self.blue, 2)
                                coeff = cv2.matchShapes(contour, piece, 1, 0.0)
                                label = str(coeff)
                                M = cv2.moments(contour)
                                cx = int(M['m10']/M['m00'])
                                cy = int(M['m01']/M['m00'])
                                textbox = cv2.rectangle(frame, (cx, cy-25), (cx + 250, cy + 10), (255,255,255), -1)
                                cv2.putText(frame, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)


                    M = cv2.moments(contour)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv2.circle(frame, (cx, cy), 5, self.red, -1)

                    # coeff = cv2.matchShapes(contour, pink_piece_contour, 1, 0.0)
                    # label = str(ratio)
                    # textbox = cv2.rectangle(frame, (cx, cy-25), (cx + 250, cy + 10), (255,255,255), -1)
                    # cv2.putText(frame, label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        # Alternatively, publish the black/white image.
        # self.pub.publish(self.bridge.cv2_to_imgmsg(binary))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('balldetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
