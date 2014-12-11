#!/usr/bin/env python
import rospy
from smach import State, StateMachine

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

        with self:
            StateMachine.add('SAY_HELLO', SpeechFromPoolSM(pool=text_pool, blocking=False), transitions={'succeeded':'NAO_HELLO_GESTURE'})
            StateMachine.add('NAO_HELLO_GESTURE', ExecuteBehavior('say_hello'), transitions={'succeeded':'succeeded'})


class WaitForObjectSegmentationState(ReadTopicState):
    def __init__(self, cluster_topic='/kinect2_clusters', timeout=3):
        ReadTopicState.__init__(self, topic_name=cluster_topic, 
                                      topic_type=PointCloudClusterCentroids,
                                      output_key_name='received_clusters', timeout=timeout)