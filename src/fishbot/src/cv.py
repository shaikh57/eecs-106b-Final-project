import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def vision() -> None:
    """
    Caller for image processing callback.
    """
    rospy.Subscriber("/usb_cam/image_raw", Image, process_image)
    rospy.spin()


def main() -> None:
    """
    Main driver for computer vision node.
    """
    # rospy.init_node("computer_vision", anonymous=True)

    # vision()

    cv_image = cv2.imread("cv_image.png")

    # downsample cv_image by a factor of 2 using nearest interpolation
    img_d = cv2.resize(src=cv_image, dsize=(cv_image.shape[1]//2, cv_image.shape[0]//2), interpolation=cv2.INTER_NEAREST)
    
    hsvFrame = cv2.cvtColor(img_d, cv2.COLOR_BGR2HSV)

    # define BGR lower and upper limits to threshold blue color
    blue_lower = np.array([100, 80, 10], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)

    # creates a mask
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # creates a kernel used to remove from mask
    kernel = np.ones((7, 7), np.uint8)

    # remove noise from the mask
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

    # creates thresholded blue image using the mask
    thresholded_img = cv2.bitwise_and(img_d, img_d, mask=blue_mask)

    # upsample the image back to the original cv_image by a factor of 2 using nearest interpolation
    img_u = cv2.resize(src=thresholded_img, dsize=(cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)
    img_u = img_u.astype(np.uint8)


    # show thresholded image
    cv2.imshow("image", img_u)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    


if __name__ == "__main__":
    main()
