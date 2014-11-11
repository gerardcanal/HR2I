#!/usr/bin/env python
import rospy
from wifibot_utils.srv import MoveToService, MoveToServiceResponse
from roswifibot.msg import speed_msg
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, sqrt
import matplotlib.pyplot as plt

################
### Variables ##
################
R = 0.07 # Radius (7cm)
L = 0.30 # Distance between wheels (30cm)
PIDw = ( 0.1,    #Kp 0.1
         0.001,  #Ki 0.02
         0.000   #Kd 0.00
    )
###############
# Constants ###
###############
WB_ODOM_TOPIC = '/wifibot/odom'
WB_SPEED_TOPIC = '/wifibot/cmd_speed'
RESET_INFO_SRV = '/wifibot/reset_odom'
ERROR_THETA_TH = 0.05
ERROR_DIST_TH = 0.035
V = 0.01 # Linear velocity
verbose = True
plot = False
SLEEP_TIME = 0.5 # seconds

class WifiBotMoveToService:
    def __init__(self):
        rospy.init_node('wifibot_go_to_server')
        rospy.Service('wb_move_to_srv', MoveToService, self.handle_move_to_service)
        rospy.loginfo('Wifibot move_to service started.')
        self._pub = rospy.Publisher(WB_SPEED_TOPIC, speed_msg, queue_size=10)
        self._running = False
        self._createdPlot = False
        rospy.wait_for_service(RESET_INFO_SRV)
        self._reset_info = rospy.ServiceProxy(RESET_INFO_SRV, Empty)

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
        #if (self._firstCall):
        #    self._firstCall = False
        #    self._goal = Pose2D(self._goal.x+odom[0].x, self._goal.y+odom[0].y, self._goal.theta)

        #odom = self.subtractOdomPose(odom, self._firstOdom)
        if verbose:
            print "Odom: "+self.str_odom(odom)
            print "Goal: Pose: "+str((self._goal.x, self._goal.y, 0.0))

        # Get w by PID
        theta_g = atan2(self._goal.y-odom[0].y, self._goal.x-odom[0].x) # Desired theta
        e_k = theta_g - odom[1] # Error
        e_k = atan2(sin(e_k), cos(e_k))


        # Check if finished
        dist = sqrt((odom[0].x-self._goal.x)**2+(odom[0].y-self._goal.y)**2)

        #raw_input('Press a key...')
        #if (dist <= ERROR_DIST_TH and abs(e_k) <= ERROR_THETA_TH): # We're have arrived at the place and have the correct orientation for the goal
        if (dist <= ERROR_DIST_TH): # We're have arrived at the place and have the correct orientation for the goal
            if self._orient: # We have to orient to the request orientation
                theta_g = atan2(sin(self._goal.theta), cos(self._goal.theta)) # To force it to be between -pi,pi
                e_k = theta_g - odom[1] # Error
                e_k = atan2(sin(e_k), cos(e_k))
                self._V = 0 # We don't want to move forward anymore
            if not self._orient or abs(e_k) <= ERROR_THETA_TH: # We're done
                self._reached = True # Make the service end
                self._subs.unregister() # No need to move the robot again
                self._V = V # Restore value

                # Plot if activated... to see the last points
                if plot:
                    self._plotPosX.append(odom[0].x)
                    self._plotPosY.append(odom[0].y)
                    self._plotE.append(e_k)
                if verbose:
                    print "Last dist:", dist, "e_k:", e_k, "theta_g:", theta_g, "theta:", odom[1], '\n'
                return

        if verbose:
            print "dist:", dist, "e_k:", e_k, "theta_g:", theta_g, "theta:", odom[1], '\n'
        # Plot if activated
        if plot:
            self._plotPosX.append(odom[0].x)
            self._plotPosY.append(odom[0].y)
            self._plotE.append(e_k)

        e_dot = e_k - self._old_e
        E = self._E + e_k
        w = PIDw[0]*e_k + dt*PIDw[1]*E + PIDw[2]*e_dot/dt
        # Save State
        self._old_e = e_k
        self._E = E

        # Move the robot
        (vl, vr) = self.uni_to_diff(self._V, w)
        self._pub.publish(vl, vr)


    def handle_move_to_service(self, req):
        self._reset_info()
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
        self._running = True
        self._old_time = rospy.get_time()
        #self._firstCall = True # To indicate this is a new call and save the first odom as point 0
        self._V = V
        if plot:
            self._plotPosX = []
            self._plotPosY = []
            self._plotE = []
            self.create_plot()

        self._subs = rospy.Subscriber(WB_ODOM_TOPIC, Odometry, self.odom_cb, queue_size=1)
        while (not self._reached): # Wait until we have reached the position
            if plot:
                self.plot()
            else:
                rospy.sleep(SLEEP_TIME)

        self._subs.unregister()
        self._running = False
        # if plot:
        #     #plt.ioff()
        #     plt.close(1)
        return MoveToServiceResponse(True)

    def uni_to_diff(self, v, w):
        ''' Converts unicycle velocity to the differential model one '''
        vr = (2*v+w*L)/(2*R)
        vl = (2*v-w*L)/(2*R)
        return (vl, vr)

    def str_odom(self, odom):
        pose = (odom[0].x, odom[0].y, odom[0].z)
        return 'Pose: ' + str(pose) + ' Orientation: ' + str(odom[1])

    def subtractOdomPose(self, odom, firstOdom):
        aux = Point()
        aux.x = odom[0].x - firstOdom[0].x
        aux.y = odom[0].y - firstOdom[0].y
        aux.z = odom[0].z - firstOdom[0].z
        return (aux, odom[1])

    def create_plot(self):
        plt.ion()
        plt.figure(1)
        #plt.switch_backend('TkAgg')
        mng = plt.get_current_fig_manager()
        mng.resize(*mng.window.maxsize())
        plt.show()
        self._createdPlot = True

    def plot(self):
        plt.figure(1)
        plt.subplot(211)
        plt.plot(self._plotPosX, self._plotPosY, 'bo', [self._goal.x], [self._goal.y], 'ro')
        plt.ylabel('Position Y')
        plt.xlabel('Position X')
        plt.title('Position plot')

        plt.subplot(212)
        plt.plot(self._plotE, 'g-')
        plt.axhline(y=0, color='r')
        plt.xlabel('e_k')
        plt.title('Error plot')

        plt.draw()
            

if __name__ == "__main__":
    srv = WifiBotMoveToService()
    srv.run_service()   