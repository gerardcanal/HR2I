#!/usr/bin/env python
import rospy
from smach import State, StateMachine, CBState

from hr2i_thesis.msg import GestureRecognitionResult, PointCloudClusterCentroids
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.tts_state import SpeechFromPoolSM
from nao_smach_utils.read_topic_state import ReadTopicState

class ReleaseNAOFromWifiBotState(ExecuteBehavior):
    def __init__(self):
        ExecuteBehavior.__init__(self, 'release_nao_from_wb')

class NaoSayHello(StateMachine):
    def __init__(self, text_pool=None):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

        if not text_pool:
            text_pool = ['Hello there!', 'Hello!', 'Hi!', 'Oh, hello!', 'olla!' 'allo!', 'Hey!']
        elif isinstance(str, text_pool):
            text_pool = [text_pool]

        with self:
            StateMachine.add('SAY_HELLO', SpeechFromPoolSM(pool=text_pool, blocking=False), transitions={'succeeded':'NAO_HELLO_GESTURE'})
            StateMachine.add('NAO_HELLO_GESTURE', ExecuteBehavior('say_hello'), transitions={'succeeded':'succeeded'})


class ReadObjectSegmentationTopic(ReadTopicState):
    def __init__(self, cluster_topic='/kinect2_clusters', timeout=3):
        ReadTopicState.__init__(self, topic_name=cluster_topic, 
                                      topic_type=PointCloudClusterCentroids,
                                      output_key_name='received_clusters', timeout=timeout)

class WaitForGestureRecognitionState(StateMachine):

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
                if ud.gesture_rec_result: # We have recognized something
                    ud.ground_point = ud.gesture_rec_result.ground_point
                    if self._gesture_received.gestureId == GestureRecognitionResult.idHello:
                        return 'hello_recognized'
                    elif self._gesture_received.gestureId == GestureRecognitionResult.idPointAt:
                        return 'pointat_recognized'
                else: # In fact should never reach this point...
                    ud.ground_point = None
                    return 'failed_recognition' # Timeouted

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
            StateMachine.add('SAY_NO_RECOGNITION', SpeechFromPoolSM(pool=text_pool, blocking=False), transitions={'succeeded':'WAIT_FOR_GESTURE'})

class WBGoToNearPointingLocation(StateMachine):

    def __init__(self, dist_to_loc=0.2):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['ground_point'])

        with self:
            def prepare_req(ud):
                pass  # TODO
            StateMachine.add('PREPARE_MOVE_REQUEST', CBState(prepare_req, input_keys=['in_ground_point'], output_keys=['out_move_req']),
                             remapping={'in_ground_point': 'ground_point', 'out_move_req': 'move_request'},
                             transitions={'succeeded': 'MOVE_TO_PLACE'})

