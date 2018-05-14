#!/usr/bin/env python
import rospy
from visualization_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import sensor_msgs.point_cloud2 as pc2
import laser_geometry
import tf
from matplotlib import path

class CornerMarker(Marker):
    def __init__(self, id, point, color):
        Marker.__init__(self)
        self.header.frame_id = "GameZoneBox"
        self.header.stamp = rospy.Time.now()
        self.ns = "gzb"
        self.id = id
        self.type = Marker.SPHERE
        self.action = Marker.ADD
        self.pose.position = point
        self.scale.x = 0.1
        self.scale.y = 0.1
        self.scale.z = 0.1
        self.color.a = 1
        self.color.g = 1 if color in ['g', 'y'] else 0
        self.color.b = 1 if color in ['b', 'y'] else 0
        self.color.r = 1 if color in ['r', 'y'] else 0
        colors = {'r': [1,0,0], 'g': [0,1,0], 'b': [0,0,1], 
                'p': [0.5,0,0.5], 'y': [0.5,0.5,0], 'o': [0.70,0.30,0]}
        self.color.r, self.color.g, self.color.b = colors[color]



class GameZoneBox(object):
    def __init__(self):
        self.cloud_pub = rospy.Publisher("/experiment/game_zone_detector/scan2", PointCloud2, queue_size=10)
        self.person_pub = rospy.Publisher("/experiment/game_zone_detector/enabled", Bool, queue_size=10, latch=True)

        # Transform Stuff
        self._center_tf = tf.TransformBroadcaster()
        self._listener = tf.TransformListener()

        self._center = Point()
        ### These are the settings ###
        # Center Point, relative to /laser frame
        self._center.x = 0.9
        self._center.y = 0.4
        # Sides of boxes
        self._length_y = 1.15
        self._length_x = 1.15

        self._points = list()
        self.shifted_points = None
        self._init_points()
        self.broadcast_transform()

        self.person_found = False

        rospy.Subscriber("/experiment/game_zone_detector/base_scan", LaserScan, lambda x: self.scan_points_cb(x))


    def _init_points(self):
        self._points.append(Point(x=-self._length_x/2.0, y=-self._length_y/2.0))
        self._points.append(Point(x=self._length_x/2.0, y=-self._length_y/2.0))
        self._points.append(Point(x=self._length_x/2.0, y=self._length_y/2.0))
        self._points.append(Point(x=-self._length_x/2.0, y=self._length_y/2.0))

    def broadcast_transform(self):
        self._center_tf.sendTransform((self._center.x, self._center.y, self._center.z),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"/GameZoneBox","/laser")

    def get_vis_markers(self):
        self.broadcast_transform()

        markers = MarkerArray()
        l = len(self._points)
        for i,p,c in zip(range(0,l),self._points,['r','g','b','o','y','p'][:l]):
            markers.markers.append(CornerMarker(i,p,c))
        return markers

    def scan_points_cb(self, points):
        self.broadcast_transform()
        # Turn laser data into 2d points, republish for debugging purposes
        # http://answers.ros.org/question/115136/python-pointcloud2-read_points-problem/
        projector = laser_geometry.LaserProjection()
        points2 = projector.projectLaser(points, 2.0)
        data_out = pc2.read_points(points2, field_names=None, skip_nans=True)
        self.cloud_pub.publish(points2)

        # We can only transfrom the laser scan into 2d points in the same frame,
        # so we need to bring the GameZoneBox to us. This should never be changing,
        # So only do the calculation once and save it.
        if not self.shifted_points:
            try:
                now = rospy.Time.now()
                older = now - rospy.Duration(.01)
                self._listener.waitForTransform("/laser","/GameZoneBox", now, rospy.Duration(1.0))
            except Exception as e:
                print e
                print "FUCK FUCK FUCK FUCK FUCK"

            stamped_points = [PointStamped(header=Header(stamp=now,
                                                        frame_id="/GameZoneBox"),
                                            point=pt) for pt in self._points]
            self.shifted_points = [self._listener.transformPoint("/laser", pt) for pt in stamped_points]

        # Are there any points in the box?
        # http://stackoverflow.com/q/8833950
        box_path = path.Path([(p.point.x,p.point.y) for p in self.shifted_points])
        pts = [(p[0],p[1]) for p in data_out]
        pts_sum = sum(box_path.contains_points(pts))
        if pts_sum > 10 and not self.person_found:
            self.person_found = True
            self.person_pub.publish(Bool(True))
        elif pts_sum == 0 and self.person_found:
            self.person_found = False
            self.person_pub.publish(Bool(False))





if __name__ == "__main__":
    rospy.init_node("game_zone_detector")
    pub = rospy.Publisher('/experiment/game_zone_detector/markers', MarkerArray, queue_size=1)
    box = GameZoneBox()

    try:
        while not rospy.is_shutdown():
            pub.publish(box.get_vis_markers())
            rospy.sleep(1) 
    except KeyboardInterrupt:
        pass

