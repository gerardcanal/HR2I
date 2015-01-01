#!/usr/bin/env python
from smach import StateMachine, CBState
from hr2i_smach_states import ReleaseNAOFromWifiBotState, SegmentBlobsPipeLine, WBMoveCloseToPoint, NaoGoToLocationInFront
from hr2i_disambiguate_sm import DisambiguateBlobs
from nao_smach_utils.read_topic_state import ReadTopicState
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior
from nao_smach_utils.tts_state import SpeechFromPoolSM
from nao_smach_utils.timeout_state import TimeOutState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

LAST_ORIENT = True
STABILIZATION_TIME = 2.5    # seconds
WB_DIST_TO_POINTING = 1.00  # metres
NAO_DIST_TO_OBJECT = 0.1    # metres


class PointAtResponseExecutionSM(StateMachine):
    ''' input keys are:
                - in_ground_point: The point in the ground plane which has been pointed by the UserWarning
                - in_NAO_riding: a boolean which indicates whether the NAO is riding the wifibot or not
        output_keys are:
                - out_NAO_riding: The result of whether the NAO is riding the wifibot or not (basically will be false after the execution)
    '''
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted', 'not_riding_wb', 'nothing_found'],
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
                             transitions={'possible': 'SAY_GOING_POINT', 'impossible': 'not_riding_wb'})

            going_pool = ['I see you are ponting there, I will go to check.', 'I saw where you pointed at!', 'Okay, I am going there now!',
                          'Let\'s go!!']
            StateMachine.add('SAY_GOING_POINT', SpeechFromPoolSM(pool=going_pool, blocking=False),
                             transitions={'succeeded': 'MOVE_TO_POINTING_PLACE', 'aborted': 'MOVE_TO_POINTING_PLACE', 'preempted': 'MOVE_TO_POINTING_PLACE'})

            StateMachine.add('MOVE_TO_POINTING_PLACE', WBMoveCloseToPoint(dist_to_loc=WB_DIST_TO_POINTING),
                             remapping={'ground_point': 'in_ground_point'},
                             transitions={'succeeded': 'WAIT_CAMERA_STABILIZATION'})

            StateMachine.add('WAIT_CAMERA_STABILIZATION', TimeOutState(timeout=STABILIZATION_TIME),
                             transitions={'succeeded': 'SEGMENT_BLOBS'})

            StateMachine.add('SEGMENT_BLOBS', SegmentBlobsPipeLine(), remapping={'segmented_clusters': 'segmented_clusters'},
                             transitions={'succeeded': 'DISAMBIGUATE', 'timeouted': 'SAY_CHECKING', 'no_object_found': 'SAY_NOT_FOUND'})

            _not_found_pool = ['I did not see anything there... Let\'s begin again.', 'I found nothing where you pointed at! Do it again',
                               'I am sorry nothing was found there... You can repeat if you wish.']
            StateMachine.add('SAY_NOT_FOUND', SpeechFromPoolSM(_not_found_pool),
                             transitions={'succeeded': 'nothing_found', 'preempted': 'nothing_found', 'aborted': 'nothing_found'})

            _pool = ['I am not seeing the objects there.', 'I have to clean my cameras, let me check again.',
                     'I am sorry, I look like blind today']
            StateMachine.add('SAY_CHECKING', SpeechFromPoolSM(_pool),
                             transitions={'succeeded': 'SEGMENT_BLOBS', 'aborted': 'SEGMENT_BLOBS', 'preempted': 'SEGMENT_BLOBS'})

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

            StateMachine.add('NAO_GO_TO_BLOB', NaoGoToLocationInFront(K=NAO_DIST_TO_OBJECT), remapping={'location_point': 'object_pose', 'alpha': 'theta_wb'},
                             transitions={'succeeded': 'GRASP_OBJECT'})

            StateMachine.add('GRASP_OBJECT', ExecuteBehavior(behavior_name='tomato_grasp'),
                             transitions={'succeeded': 'succeeded', 'aborted': 'succeeded', 'preempted': 'succeeded'})
