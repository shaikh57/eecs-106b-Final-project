import cv2
import rospy

from sensor_msgs.msg import Image


NUM_CLUSTERS = 2

def process_image(img: Image) -> None:
    pub = rospy.Publisher("/image_segmented", Image, queue_size=10)
    r = rospy.Rate(10)

    cv2.km

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
