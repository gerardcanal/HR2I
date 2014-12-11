import rospy
from smach import State

from hr2i_thesis.msg import GestureRecognitionResult, PointCloudClusterCentroids

class WaitForObjectSegmentationState(State):
    def __init__(self, cluster_topic='/kinect2_clusters', timeout=3):
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['received_clusters'])
        self._topic = cluster_topic
        self._timeout = rospy.Duration(timeout)
        self._clusters_info = None

    def gesture_cb(self, data):
        self._clusters_info = data

    def execute(self, ud):
        subs = rospy.Subscriber(self._topic, PointCloudClusterCentroids, self.blob_cb)
        timeout = False
        startT = rospy.Time.now()
        while not (timeout or (self._clusters_info)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._clusters_info:
            ud.received_clusters = self._clusters_info
            return 'succeeded'
        else:
            ud._clusters_info = None
            return 'aborted' # Timeouted

class WaitForGestureRecognitionState(State):
    def __init__(self, gesture_topic='/recognized_gesture', timeout=None):
        State.__init__(self, outcomes=['pointat_recognized', 'hello_recognized', 'failed_recognition'],
                       output_keys=['out_ground_point'])
        self._topic = gesture_topic
        self._timeout = rospy.Duration(timeout) if timeout else None
        self._gesture_received = None

    def gesture_cb(self, data):
        self._gesture_received = data

    def execute(self, ud):
        subs = rospy.Subscriber(self._topic, GestureRecognitionResult, self.gesture_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._gesture_received)):
            timeout = False if self._timeout is None else (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._gesture_received:
            ud.ground_point = self._gesture_received.ground_point
            if self._gesture_received.gestureId == self._gesture_received.idHello:
                return 'hello_recognized'
            elif self._gesture_received.gestureId == self._gesture_received.idPointAt:
                return 'pointat_recognized'
        else:
            ud.ground_point = None
            return 'failed_recognition' # Timeouted