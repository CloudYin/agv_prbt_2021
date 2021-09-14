
#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np


def get_blue_marker_pose(calibrated_file_path):
    original_image = cv2.imread(calibrated_file_path)
    image_copy = original_image.copy()
    image_hsv = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)
    lower_b = np.array([100, 100, 50])
    upper_b = np.array([120, 255, 255])
    mask = cv2.inRange(image_hsv, lower_b, upper_b)
    cv2.bitwise_and(original_image, original_image, mask=mask)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    # Select inner contours
    hierarchy = hierarchy[0]
    for component in zip(contours, hierarchy):
        currentContour = component[0]
        currentHierarchy = component[1]
        if currentHierarchy[3] != -1:
            # these are the innermost child components
            img_contour = cv2.drawContours(image_copy, currentContour, -1, (0, 0, 0))
            if cv2.moments(currentContour)['m00'] > 500:
                m00 = cv2.moments(currentContour)['m00']
                m01 = cv2.moments(currentContour)['m01']
                m10 = cv2.moments(currentContour)['m10']
                centerX = round(m10/m00)
                centerY = round(m01/m00)
                rect = cv2.minAreaRect(currentContour)
                points = cv2.boxPoints(rect)
                points = np.int0(points)
                cv2.drawContours(img_contour, [points], -1, (0, 255, 0), 2)
                angle = -cv2.minAreaRect(currentContour)[2]
                if angle > 45:
                    angle = angle - 90
                cv2.circle(img_contour, (int(centerX), int(centerY)), 1, (0, 0, 255))
    # cv2.imshow("img_contour", reshaped_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return (centerX, centerY, angle)


if __name__ == '__main__':
    calibrated_file_path = '/home/pilz/Pictures/agv_prbt/table_calibrated.png'
    x, y, angles = get_blue_marker_pose(calibrated_file_path)
    print(x, y, angles)