#!/usr/bin/env python
import rospy
from wifibot_utils.srv import MoveToService, MoveToServiceResponse
from roswifibot.msg import speed_msg
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, sqrt

################
### Variables ##
################
R = 0.07 # Radius (7cm)
L = 0.30 # Distance between wheels (30cm)
PIDw = ( 0.01, #Kp
         0.001,  #Ki
         0.001   #Kd
    )
###############
# Constants ###
###############
WB_ODOM_TOPIC = '/wifibot/odom'
WB_SPEED_TOPIC = '/wifibot/cmd_speed'
ERROR_THETA_TH = 0.1
ERROR_DIST_TH = 0.01
V = 0.005 # Linear velocity
verbose = False
SLEEP_TIME = 0.25 # seconds

class WifiBotMoveToService:
    def __init__(self):
        rospy.init_node('wifibot_go_to_server')
        rospy.Service('wb_move_to_srv', MoveToService, self.handle_move_to_service)
        rospy.loginfo('Wifibot move_to service started.')
        self._pub = rospy.Publisher(WB_SPEED_TOPIC, speed_msg, queue_size=10)
        self._running = False

    def run_service(self):
        rospy.spin()

    def odom_cb(self, data):
        # Get time step
        t = rospy.get_time()
        dt = t - self._old_time
        self._old_time = t
        
        #Start control
        quaternion = data.pose.pose.orientation
        odom = (data.pose.pose.position, euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[2])
        if (not self._firstOdom):
            self._firstOdom = (Point(odom[0].x, odom[0].y, odom[0].z), odom[1])

        odom = self.subtractOdom(odom, self._firstOdom)
        print "Odom: "+self.str_odom(odom)
        if (verbose): rospy.loginfo('Normalized topic /odom is %s' % self.str_odom(odom))

        # Get w by PID
        theta_g = atan2(self._goal.y-odom[0].y, self._goal.x-odom[0].x) # Desired theta
        e_k = theta_g - odom[1] # Error
        e_k = atan2(sin(e_k), cos(e_k))


        # Check if finished
        dist = sqrt((odom[0].x-self._goal.x)**2+(odom[0].y-self._goal.y)**2)
        print "dist:", dist, "e_k:", e_k, " theta_g:", theta_g, "theta:", odom[1]
        raw_input('Enter input:')
        if dist <= ERROR_DIST_TH and abs(e_k) <= ERROR_THETA_TH: # We're have arrived at the place and have the correct orientation for the goal
            if self._orient: # We have to orient to the request orientation
                theta_g = atan2(sin(self._goal.theta), cos(self._goal.theta)) # To force it to be between -pi,pi
                e_k = theta_g - odom[1] # Error
                e_k = atan2(sin(e_k), cos(e_k))
            if abs(e_k) <= ERROR_THETA_TH: # We're done
                self._reached = True # Make the service end
                self._subs.unregister() # No need to move the robot again
                return

        e_dot = e_k - self._old_e
        E = self._E + e_k
        w = PIDw[0]*e_k + dt*PIDw[1]*E + PIDw[2]*e_dot/dt
        w = PIDw[0]*e_k
        # Save State
        self._old_e = e_k
        self._E = E

        # Move the robot
        #w = 0;
        (vl, vr) = self.uni_to_diff(V, w)
        #print w, vl, vr, dt
        print
        self._pub.publish(vl, vr)


    def handle_move_to_service(self, req):
        if (req.orient): rospy.loginfo("Received call to move wifibot to ORIENTED position (%f, %f, %f)..." % (req.pose.x, req.pose.y, req.pose.theta))
        else: rospy.loginfo("Received call to move wifibot to position (%f, %f, %f)..." % (req.pose.x, req.pose.y, req.pose.theta))
        if (self._running):
            rospy.logerr("Called service while already running!")
            return False
        # Init variables
        self._reached = False
        self._goal = req.pose
        self._orient = req.orient
        self._old_e = 0.0
        self._E = 0.0
        self._firstOdom = None # To indicate this is a new call and save the first odom as point 0
        self._running = True
        self._old_time = rospy.get_time()

        self._subs = rospy.Subscriber(WB_ODOM_TOPIC, Odometry, self.odom_cb, queue_size=1)
        while (not self._reached): # Wait until we have reached the position
            rospy.sleep(SLEEP_TIME)
        self._subs.unregister()
        self._running = False
        return MoveToServiceResponse(True)

    def uni_to_diff(self, v, w):
        ''' Converts unicycle velocity to the differential model one '''
        vr = (2*v+w*L)/(2*R)
        vl = (2*v-w*L)/(2*R)
        return (vl, vr)

    def str_odom(self, odom):
        pose = (odom[0].x, odom[0].y, odom[0].z)
        return 'Pose: ' + str(pose) + ' Orientation: ' + str(odom[1])

    def subtractOdom(self, odom, firstOdom):
        aux = Point()
        aux.x = odom[0].x - firstOdom[0].x
        aux.y = odom[0].y - firstOdom[0].y
        aux.z = odom[0].z - firstOdom[0].z
        return (aux, odom[1] - firstOdom[1])

    def addOdom(self, odom, firstOdom):
        aux = Point()
        aux.x = odom[0].x + firstOdom[0].x
        aux.y = odom[0].y + firstOdom[0].y
        aux.z = odom[0].z + firstOdom[0].z
        return (aux, odom[1] + firstOdom[1])


if __name__ == "__main__":
    srv = WifiBotMoveToService()
    srv.run_service()   