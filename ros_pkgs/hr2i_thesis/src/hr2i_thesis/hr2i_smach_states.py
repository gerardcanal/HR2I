#!/usr/bin/env python
import rospy
import math

from smach import StateMachine, CBState
from hr2i_thesis.msg import GestureRecognitionResult, PointCloudClusterCentroids
from wifibot_utils.srv import MoveToServiceRequest
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.tts_state import SpeechFromPoolSM
from nao_smach_utils.read_topic_state import ReadTopicState
from wifibot_smach.wifibot_goto_state import GoToPositionState


class ReleaseNAOFromWifiBotState(ExecuteBehavior):
    ''' Output key NAO_riding set to false to notify that NAO has jumped down of the Wifibot '''
    def __init__(self):
        ExecuteBehavior.__init__(self, 'release_nao_from_wb')
        self.register_output_keys(['NAO_riding'])

    def execute(self, userdata):  # Override so the output key is set properly for this specific test
        super_outcome = super(ReleaseNAOFromWifiBotState, self).execute(userdata)
        userdata.NAO_riding = False
        return super_outcome


class NaoSayHello(StateMachine):
    def __init__(self, text_pool=None):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

        if not text_pool:
            text_pool = ['Hello there!', 'Hello!', 'Hi!', 'Oh, hello!', 'olla!' 'allo!', 'Hey!']
        elif isinstance(str, text_pool):
            text_pool = [text_pool]

        with self:
            StateMachine.add('SAY_HELLO', SpeechFromPoolSM(pool=text_pool, blocking=False), transitions={'succeeded': 'NAO_HELLO_GESTURE'})
            StateMachine.add('NAO_HELLO_GESTURE', ExecuteBehavior('say_hello'), transitions={'succeeded': 'succeeded'})


class ReadObjectSegmentationTopic(ReadTopicState):
    def __init__(self, cluster_topic='/kinect2_clusters', timeout=3):
        ReadTopicState.__init__(self, topic_name=cluster_topic,
                                topic_type=PointCloudClusterCentroids,
                                output_key_name='received_clusters', timeout=timeout)


class WaitForGestureRecognitionSM(StateMachine):

    def __init__(self):
        StateMachine.__init__(self, outcomes=['pointat_recognized', 'hello_recognized'], output_keys=['out_ground_point'])

        with self:
            StateMachine.add('WAIT_FOR_GESTURE', ReadTopicState(topic_name='/recognized_gesture',
                                                                topic_type=GestureRecognitionResult,
                                                                output_key_name='gesture_rec_result',
                                                                timeout=30),
                             remapping={'gesture_rec_result': 'gesture_rec_result'},
                             transitions={'succeeded': 'CHECK_GESTURE_RESULT', 'timeouted': 'SAY_NO_RECOGNITION'})

            def check_result_cb(ud):
                if ud.gesture_rec_result:  # We have recognized something
                    ud.ground_point = ud.gesture_rec_result.ground_point
                    if self._gesture_received.gestureId == GestureRecognitionResult.idHello:
                        return 'hello_recognized'
                    elif self._gesture_received.gestureId == GestureRecognitionResult.idPointAt:
                        return 'pointat_recognized'
                else:  # In fact should never reach this point...
                    ud.ground_point = None
                    return 'failed_recognition'  # Timeouted

            StateMachine.add('CHECK_GESTURE_RESULT', CBState(check_result_cb,
                                                             input_keys=['gesture_rec_result'],
                                                             output_keys=['ground_point'],
                                                             outcomes=['pointat_recognized', 'hello_recognized', 'failed_recognition']),
                             remapping={'gesture_rec_result': 'gesture_rec_result', 'ground_point': 'out_ground_point'},
                             transitions={'pointat_recognized': 'pointat_recognized',
                                          'hello_recognized': 'hello_recognized',
                                          'failed_recognition': 'SAY_NO_RECOGNITION'})

            text_pool = ['I did not see you moving. Are you there?', 'I could not see any gesture I understand.', 'Please, do a gesture',
                         'I am waiting for you to move', 'Shake your right arm!', 'This is about gesture recognition',
                         'I understand the Point at gesture and the hello one']
            StateMachine.add('SAY_NO_RECOGNITION', SpeechFromPoolSM(pool=text_pool, blocking=False), transitions={'succeeded': 'WAIT_FOR_GESTURE'})


def _Pose2D_to_str(pose):
    # Small function to get a str from a Pose2D in a single line for displaying purposes.
    # Maybe should be moved to some better place so it can be used in more scripts
    return '(' + str(pose.x) + ', ' + str(pose.y) + ', ' + str(pose.theta) + ')'


class WBGoToLocationNearPoint(StateMachine):

    def __init__(self, dist_to_loc=0.2):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'], input_keys=['ground_point'])

        with self:
            def prepare_req(ud):
                # Compute the ground_point pose but displaced dist_to_loc metres in the direction of the robot
                # To do so an equation system has been solved with the following equations (Assuming robot at 0,0:
                #   Line equation: y = (gy/gx) * x -- being (gx, gy) the ground point
                #   Pitagoras: k^2 = a^2 + b^2 -- being k = dist_to_loc   . (xg, yg)
                #   x = gx - a  --                                       /|
                #   y = gy - b  --                                      / | <- a
                #                                                      /__|
                #                                                (x,y).  ^b
                _a = math.sqrt((dist_to_loc**2) / (((-float(ud.in_ground_point.y)/ud.in_ground_point.x)**2) + 1))
                _b = math.sqrt(dist_to_loc**2 - _a**2)
                req = MoveToServiceRequest()
                req.pose.x = ud.in_ground_point.x - _a
                req.pose.y = ud.in_ground_point.y - _b
                req.orient = False
                ud.out_move_req = req
                rospy.loginfo('--WBGoToLocationNearPoint state prepare req -- Original pose: ' +
                              _Pose2D_to_str(ud.in_ground_point) + ', pose at ' + str(dist_to_loc) + ' metres: ' +
                              _Pose2D_to_str(req.pose))
                return 'succeeded'

            StateMachine.add('PREPARE_MOVE_REQUEST', CBState(prepare_req, input_keys=['in_ground_point'],
                                                             output_keys=['out_move_req'],
                                                             outcomes=['succeeded']),
                             remapping={'in_ground_point': 'ground_point', 'out_move_req': 'move_request'},
                             transitions={'succeeded': 'MOVE_TO_PLACE'})

            StateMachine.add('MOVE_TO_PLACE', GoToPositionState(), remapping={'request': 'move_request'},
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})
