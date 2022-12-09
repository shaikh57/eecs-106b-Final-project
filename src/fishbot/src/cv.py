import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


NUM_CLUSTERS = 2

def process_image(img: Image) -> None:
    pub = rospy.Publisher("/image_segmented", Image, queue_size=10)
    r = rospy.Rate(10)

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

    # Downsample img by a factor of 2 first using the mean to speed up K-means
    img_d = cv2.resize(src=cv_image, dsize=(cv_image.shape[1]//2, cv_image.shape[0]//2), interpolation=cv2.INTER_NEAREST)

    hsvFrame = cv2.cvtColor(img_d, cv2.COLOR_BGR2HSV)

    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = cv2.resize(src=blue_mask, dsize=(cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_NEAREST)
    
    img_u.astype(np.uint8)



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
    rospy.init_node("computer_vision", anonymous=True)

    vision()


if __name__ == "__main__":
    main()
