#!/usr/bin/env python
import rospy
from wifibot_utils.srv import MoveToService, MoveToServiceResponse
from roswifibot.msg import speed_msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos

################
### Variables ##
################
R = 0.07 # Radius (7cm)
L = 0.30 # Distance between wheels (30cm)
PIDw = ( 10.0, #Kp
         0.1,  #Ki
         0.1   #Kd
    )
###############
# Constants ###
###############
WB_ODOM_TOPIC = '/wifibot/odom'
WB_SPEED_TOPIC = '/wifibot/cmd_speed'
ERROR_TH = 0.05
V = 1.0 # Linear velocity
verbose = False
SLEEP_TIME = 0.25 # seconds

class WifiBotMoveToService:
    def __init__(self):
        rospy.init_node('wifibot_go_to_server')
        rospy.Service('wb_move_to_srv', MoveToService, self.handle_move_to_service)
        rospy.loginfo('Wifibot move_to service started.')
        self._pub = rospy.Publisher(WB_SPEED_TOPIC, speed_msg, queue_size=10)

    def run_service(self):
        rospy.spin()

    def odom_cb(self, data):
        quaternion = data.pose.pose.orientation
        self._odom = (data.pose.pose.position, euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[2])
        if (verbose): rospy.loginfo('Topic /odom is %s' % self.str_odom(self._odom))

        # Get w by PID
        theta_g = atan2(self._goal.y-self._odom[0].y, self._goal.x-self._odom[0].x) # Desired theta
        if theta_g <= ERROR_TH: # FIXME check!
            theta_g = self._goal.theta
        e_k = theta_g - self._odom[1] # Error
        e_k = atan2(sin(e_k), cos(e_k))
        if e_k <= ERROR_TH: # We're done
            self._reached = True # Make the service end
            self._subs.unregister() # No need to move the robot again
            return
        e_dot = e_k - self._old_e
        E = self._E + e_k
        w = PIDw[0]*e_k + PIDw[1]*E + PIDw[2]*e_dot

        # Save State
        self._old_e = e_k
        self._E = E

        # Move the robot
        (vl, vr) = self.uni_to_diff(V, w)
        self._pub.publish(vl, vr)


    def handle_move_to_service(self, req):
        rospy.loginfo("Received call to move wifibot to position (%f, %f, %f)..." % (req.pose.x, req.pose.y, req.pose.theta))
        # Init variables
        self._reached = False
        self._goal = req.pose
        self._old_e = 0.0
        self._E = 0.0

        self._subs = rospy.Subscriber(WB_ODOM_TOPIC, Odometry, self.odom_cb)
        while (not self._reached): # Wait until we have reached the position
            rospy.sleep(SLEEP_TIME)
        self._subs.unregister()

        return MoveToServiceResponse(True)

    def uni_to_diff(self, v, w):
        ''' Converts unicycle velocity to the differential model one '''
        vr = (2*v+w*L)/(2*R)
        vl = (2*v-w*L)/(2*R)
        return (vl, vr)

    def str_odom(self, odom):
        pose = (odom[0].x, odom[0].y, odom[0].z)
        return 'Pose: ' + str(pose) + ' Orientation: ' + str(odom[1])


if __name__ == "__main__":
    srv = WifiBotMoveToService()
    srv.run_service()   