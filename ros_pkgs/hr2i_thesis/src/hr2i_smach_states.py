#!/usr/bin/env python
import rospy
from smach import State, StateMachine

from hr2i_thesis.msg import GestureRecognitionResult
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.tts_state import SpeechState

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

class ReleaseNAOFromWifiBotState(ExecuteBehavior):
    def __init__(self):
        ExecuteBehavior.__init__(self, 'release_nao_from_wb')

class NaoSayHello(StateMachine):
    def __init__(self, text):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted'])

        with self:
            StateMachine.add('SAY_HELLO', SpeechState)

class WaitForObjectBlobsState(State):
    def __init__(self, blob_topic='/kinect2_blobs', timeout=3):
        State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['blobsFIXME'])
        self._topic = blob_topic
        self._timeout = rospy.Duration(timeout)
        self._blob_received = None

    def gesture_cb(self, data):
        self._blob_received = data

    def execute(self, ud):
        subs = rospy.Subscriber(self._topic, 'FIXME BlobResult', self.blob_cb)
        startT = rospy.Time.now()
        timeout = False
        while not (timeout or (self._gesture_received)):
            timeout = (rospy.Time.now()-startT) > self._timeout

        subs.unregister()

        if self._gesture_received:
            '''TODO'''
            return '''TODO'''
        else:
            ud.TODO = None
            return 'aborted' # Timeouted

