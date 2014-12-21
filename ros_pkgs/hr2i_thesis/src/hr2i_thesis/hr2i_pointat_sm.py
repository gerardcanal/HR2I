#!/usr/bin/env python
from smach import StateMachine, CBState
from hr2i_smach_states import ReadObjectSegmentationTopic, ReleaseNAOFromWifiBotState, WBMoveCloseToPoint, NaoGoToLocationInFront
from disambiguate_sm import DisambiguateBlobs
from nao_smach_utils.read_topic_state import ReadTopicState
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.tts_state import SpeechFromPoolSM
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

LAST_ORIENT = True


class PointAtResponseExecutionSM(StateMachine):
    ''' input keys are:
                - in_ground_point: The point in the ground plane which has been pointed by the UserWarning
                - in_NAO_riding: a boolean which indicates whether the NAO is riding the wifibot or not
        output_keys are:
                - out_NAO_riding: The result of whether the NAO is riding the wifibot or not (basically will be false after the execution)
    '''
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'not_riding_wb'],
                              input_keys=['in_ground_point', 'in_NAO_riding'], output_keys=['out_NAO_riding'])
        with self:
            def check_on_wb(ud):
                if ud.in_NAO_riding:  # NAO is riding the wifibot, we can go
                    return 'possible'
                # Set to the same as the status has not been changed
                ud.out_NAO_riding = ud.in_NAO_riding
                return 'impossible'

            StateMachine.add('CHECK_IF_POSSIBLE', CBState(check_on_wb, input_keys=['in_NAO_riding'],
                                                          output_keys=['out_NAO_riding'],
                                                          outcomes=['possible', 'impossible']),
                             remapping={'in_NAO_riding': 'in_NAO_riding', 'out_NAO_riding': 'out_NAO_riding'},
                             transitions={'possible': 'MOVE_TO_POINTING_PLACE', 'impossible': 'not_riding_wb'})

            StateMachine.add('MOVE_TO_POINTING_PLACE', WBMoveCloseToPoint(dist_to_loc=0.2), transitions={'succeeded': 'WAIT_FOR_BLOBS'},
                             remapping={'request': 'move_to_request'})

            StateMachine.add('WAIT_FOR_BLOBS', ReadObjectSegmentationTopic(), remapping={'received_clusters': 'segmented_clusters'},
                             transitions={'succeeded': 'DISAMBIGUATE', 'timeouted': 'SAY_CHECKING'})

            _pool = ['I am not seeing the objects there...', 'I have to clean my cameras, let me check again...',
                     'I am sorry, I look like blind today']
            StateMachine.add('SAY_CHECKING', SpeechFromPoolSM(_pool),
                             transitions={'succeeded': 'WAIT_FOR_BLOBS', 'aborted': 'WAIT_FOR_BLOBS', 'preempted': 'WAIT_FOR_BLOBS'})

            StateMachine.add('DISAMBIGUATE', DisambiguateBlobs(),
                             remapping={'info_clusters': 'segmented_clusters', 'ground_point': 'in_ground_point', 'selected_cluster_centroid': 'object_pose'},
                             transitions={'succeeded': 'RELEASE_NAO'})

            StateMachine.add('RELEASE_NAO', ReleaseNAOFromWifiBotState(), remapping={'NAO_riding_wb': 'out_NAO_riding'},
                             transitions={'succeeded': 'GET_ODOM', 'preempted': 'GET_ODOM'})

            StateMachine.add('GET_ODOM', ReadTopicState(topic_name='/wifibot/odom', topic_type=Odometry, output_key_name='wb_odom', timeout=None),
                             transitions={'succeeded': 'EXTRACT_THETA', 'timeouted': 'GET_ODOM'})

            def get_theta(ud):
                quaternion = ud.wb_odom.pose.pose.orientation
                ud.theta_wb = euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[2]
                return 'succeeded'
            StateMachine.add('EXTRACT_THETA', CBState(get_theta, outcomes=['succeeded'], input_keys=['wb_odom'], output_keys=['theta_wb']),
                             transitions={'succeeded': 'NAO_GO_TO_BLOB'})

            StateMachine.add('NAO_GO_TO_BLOB', NaoGoToLocationInFront(K=0.2), remapping={'location_point': 'object_pose', 'alpha': 'theta_wb'},
                             transitions={'succeeded': 'GRASP_OBJECT'})

            StateMachine.add('GRASP_OBJECT', ExecuteBehavior(behavior_name='tomato_grasp'),
                             transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 'preempted': 'succeeded'})
