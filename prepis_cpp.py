import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from sklearn.cluster import DBSCAN
import numpy as np


class PointXYZ:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def cluster_point_cloud(points, eps, min_samples):
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_

    clusters = []
    for label in set(labels):
        if label == -1:
            continue
        clusters.append(
            [PointXYZ(points[i, 0], points[i, 1], points[i, 2]) for i in range(len(points)) if labels[i] == label])
    return clusters


def point_cloud_callback(msg):
    global prejsnji_center_x1, prejsnji_center_y1, prejsnji_center_x2, prejsnji_center_y2
    global stevec_levo, stevec_desno
    global flag_desno, flag_levo

    points_list = np.array(
        [[point[0], point[1], point[2]] for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])

    if points_list.size == 0:
        return

    eps = 0.01
    min_samples = 10
    clusters = cluster_point_cloud(points_list, eps, min_samples)

    cluster_centers = []
    for cluster in clusters:
        center_x = sum([point.x for point in cluster]) / len(cluster)
        center_y = sum([point.y for point in cluster]) / len(cluster)
        cluster_centers.append(PointXYZ(center_x, center_y, 0))

    point1 = PointXYZ(0.2, 0.4, 0.0)
    point2 = PointXYZ(0.2, -0.4, 0.0)

    def distance(p1, p2):
        return np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    min_distance1 = float('inf')
    min_distance2 = float('inf')
    closest_center1 = None
    closest_center2 = None

    for center in cluster_centers:
        d1 = distance(point1, center)
        if d1 < min_distance1:
            min_distance1 = d1
            closest_center1 = center

        d2 = distance(point2, center)
        if d2 < min_distance2:
            min_distance2 = d2
            closest_center2 = center

    if closest_center1.y < 0:
        closest_center1.x = prejsnji_center_x1
        closest_center1.y = prejsnji_center_y1

    if closest_center2.y > 0:
        closest_center2.x = prejsnji_center_x2
        closest_center2.y = prejsnji_center_y2

    if flag_levo:
        if abs(prejsnji_center_x1 - closest_center1.x) < 0.01:
            stevec_levo += 1
            flag_levo = False

    if flag_desno:
        if abs(prejsnji_center_x2 - closest_center2.x) < 0.01:
            stevec_desno += 1
            flag_desno = False

    if abs(prejsnji_center_x1 - closest_center1.x) > 0.05:
        flag_levo = True

    if abs(prejsnji_center_x2 - closest_center2.x) > 0.05:
        flag_desno = True

    prejsnji_center_x1 = closest_center1.x
    prejsnji_center_y1 = closest_center1.y
    prejsnji_center_x2 = closest_center2.x
    prejsnji_center_y2 = closest_center2.y

    marker1 = Marker()
    marker1.header.frame_id = "velodyne"
    marker1.header.stamp = rospy.Time.now()
    marker1.ns = "cluster_centers"
    marker1.id = 1
    marker1.type = Marker.SPHERE
    marker1.action = Marker.ADD
    marker1.pose.position.x = closest_center1.x
    marker1.pose.position.y = closest_center1.y
    marker1.pose.position.z = 0
    marker1.pose.orientation.x = 0.0
    marker1.pose.orientation.y = 0.0
    marker1.pose.orientation.z = 0.0
    marker1.pose.orientation.w = 1.0
    marker1.scale.x = 0.1
    marker1.scale.y = 0.1
    marker1.scale.z = 0.1
    marker1.color.r = 1.0
    marker1.color.g = 0.0
    marker1.color.b = 0.0
    marker1.color.a = 1.0

    marker2 = Marker()
    marker2.header.frame_id = "velodyne"
    marker2.header.stamp = rospy.Time.now()
    marker2.ns = "cluster_centers"
    marker2.id = 2
    marker2.type = Marker.SPHERE
    marker2.action = Marker.ADD
    marker2.pose.position.x = closest_center2.x
    marker2.pose.position.y = closest_center2.y
    marker2.pose.position.z = 0
    marker2.pose.orientation.x = 0.0
    marker2.pose.orientation.y = 0.0
    marker2.pose.orientation.z = 0.0
    marker2.pose.orientation.w = 1.0
    marker2.scale.x = 0.1
    marker2.scale.y = 0.1
    marker2.scale.z = 0.1
    marker2.color.r = 0.0
    marker2.color.g = 1.0
    marker2.color.b = 0.0
    marker2.color.a = 1.0

    pub_marker.publish(marker1)
    pub_marker.publish(marker2)

    print("Stevilo koruz na levi:", stevec_levo)
    print("Stevilo koruz na desni:", stevec_desno)


if __name__ == "__main__":
    rospy.init_node('stetje_koruzinatorja_urban', anonymous=True)

    pub_marker = rospy.Publisher('visualization_marker_urban', Marker, queue_size=1)

    prejsnji_center_x1, prejsnji_center_y1 = 0.0, 0.4
    prejsnji_center_x2, prejsnji_center_y2 = 0.0, -0.4
    stevec_levo, stevec_desno = 0, 0
    flag_desno, flag_levo = False, False

    rospy.Subscriber('/projected_omejen', PointCloud2, point_cloud_callback)

    rospy.spin()
