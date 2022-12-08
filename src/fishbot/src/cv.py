import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image


NUM_CLUSTERS = 2

def process_image(img: Image) -> None:
    pub = rospy.Publisher("/image_segmented", Image, queue_size=10)
    r = rospy.Rate(10)


def do_kmeans(data, n_clusters):
    """Uses opencv to perform k-means clustering on the data given. Clusters it into
       n_clusters clusters.
       Args:
         data: ndarray of shape (n_datapoints, dim)
         n_clusters: int, number of clusters to divide into.
       Returns:
         clusters: integer array of length n_datapoints. clusters[i] is
         a number in range(n_clusters) specifying which cluster data[i]
         was assigned to. 
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    _, clusters, centers = kmeans = cv2.kmeans(data.astype(np.float32), n_clusters, bestLabels=None, criteria=criteria, attempts=1, flags=cv2.KMEANS_RANDOM_CENTERS)

    return clusters

def cluster_segment(img, n_clusters, random_state=0):
    """segment image using k_means clustering
    Parameter
    ---------
    img : ndarray
        rgb image array
    n_clusters : int
        the number of clusters to form as well as the number of centroids to generate
    random_state : int
        determines random number generation for centroid initialization
    Returns
    -------
    ndarray
        clusters of gray_img represented with similar pixel values
    """
    # Remove this line when you implement this function.
    # raise NotImplementedError()

    # Downsample img by a factor of 2 first using the mean to speed up K-means
    img_d = cv2.resize(img, dsize=(img.shape[1]//2, img.shape[0]//2), interpolation=cv2.INTER_NEAREST)

    # TODO: Generate a clustered image using K-means

    # first convert our 3-dimensional img_d array to a 2-dimensional array
    # whose shape will be (height * width, number of channels) hint: use img_d.shape
    img_r = np.reshape(img_d, (img_d.shape[0] * img_d.shape[1], 3))
    
    # fit the k-means algorithm on this reshaped array img_r using the
    # the do_kmeans function defined above.
    clusters = do_kmeans(img_r, n_clusters)

    # reshape this clustered image to the original downsampled image (img_d) width and height 
    cluster_img = np.reshape(clusters, (img_d.shape[0], img_d.shape[1]))

    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = cv2.resize(src=cluster_img, dsize=(img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)

    return img_u.astype(np.uint8)


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
