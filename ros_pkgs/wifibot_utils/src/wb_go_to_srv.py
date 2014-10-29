#!/usr/bin/env python
import rospy
from wifibot_utils.srv import MoveToService, MoveToServiceResponse
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

R = 0.07 # Radius 7cm
L = 0.00 # Istance between wheels

WB_ODOM_TOPIC = '/odom'

class WifiBotMoveToService:
    def __init__(self):
        rospy.init_node('wifibot_go_to_server')
        rospy.Service('wb_move_to_srv', MoveToService, self.handle_move_to_service)
        self.subscribe_odom()
        rospy.loginfo('Wifibot move_to service started.')

    def run_service(self):
        rospy.spin()

    def subscribe_odom(self):
        self._subs = rospy.Subscriber(WB_ODOM_TOPIC, Odometry, self.odom_cb)

    def odom_cb(self, data):
        quaternion = data.pose.pose.orientation
        self._odom = (data.pose.pose.position, euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[2])
        rospy.loginfo('Topic /odom is %s' % self.str_odom(self._odom))

    def str_odom(self, odom):
        pose = (odom[0].x, odom[0].y, odom[0].z)
        return 'Pose: ' + str(pose) + ' Orientation: ' + str(odom[1])

    def handle_move_to_service(self, req):
        rospy.loginfo("Received call to move wifibot to position (%f, %f, %f)..." % (req.pose.x, req.pose.y, req.pose.theta))
        self._subs.unregister() # Unregister as we don't care about the /odom while moving... I think
        currState = self._odom
        rospy.loginfo('Odom is %s' % self.str_odom(currState))
        self.subscribe_odom()
        return MoveToServiceResponse(True)



if __name__ == "__main__":
    srv = WifiBotMoveToService()
    srv.run_service()   