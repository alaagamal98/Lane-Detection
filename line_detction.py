#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
import math

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print ("Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir))
    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        # print(msg)
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        line_image = DetectLine(cv_img)
        cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), line_image)
        print ("Wrote image %i" % count)

        count += 1

    bag.close()

    return
def DetectLine(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
    low_threshold = 50
    high_threshold = 150
    edges = cv2.Canny(blur_gray, low_threshold, high_threshold)
    rho = 1  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 15  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 50  # minimum number of pixels making up a line
    max_line_gap = 20  # maximum gap in pixels between connectable line segments
    line_image = np.copy(img) * 0  # creating a blank to draw lines on

    # Run Hough on edge detected image
    # Output "lines" is an array containing endpoints of detected line segments
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
                        min_line_length, max_line_gap)
    max_dist = -1.0
    for line in lines:
        for x1, y1, x2, y2 in line:
            theta1 = (y2-y1)
            theta2 = (x2-x1)
            hyp = math.hypot(theta1, theta2)
            if (max_dist < hyp):
                max_l = line
                max_dist = hyp
    for x1, y1, x2, y2 in max_l:        
        cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
            
    lines_edges = cv2.addWeighted(img, 0.8, line_image, 1, 0)
    plt.imshow(lines_edges, cmap='gray')
    plt.show()
    return line_image

if __name__ == '__main__':
    main()