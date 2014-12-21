#!/usr/bin/env python
import rospy
import math

from smach import StateMachine, CBState, Concurrence
from hr2i_thesis.msg import GestureRecognitionResult, PointCloudClusterCentroids
from wifibot_utils.srv import MoveToServiceRequest
from nao_smach_utils.execute_choregraphe_behavior_state import ExecuteBehavior, _checkInstalledBehavior
from nao_smach_utils.tts_state import SpeechFromPoolSM
from nao_smach_utils.read_topic_state import ReadTopicState
from nao_smach_utils.move_to_state import MoveToState
from wifibot_smach.wifibot_goto_state import GoToPositionState
from geometry_msgs.msg import Pose2D


class ReleaseNAOFromWifiBotState(ExecuteBehavior):
    ''' Output key NAO_riding set to false to notify that NAO has jumped down of the Wifibot '''
    def __init__(self):
        ExecuteBehavior.__init__(self, 'HR2I_release_nao_from_wb')
        self.register_output_keys(['NAO_riding_wb'])

    def execute(self, userdata):  # Override so the output key is set properly for this specific test
        super_outcome = super(ReleaseNAOFromWifiBotState, self).execute(userdata)
        userdata.NAO_riding_wb = False
        return super_outcome


class NaoSayHello(Concurrence):
    def __init__(self, text_pool=None):
        Concurrence.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['riding_wifibot'],
                             default_outcome='succeeded', outcome_map={'succeeded': {'SAY_HELLO': 'succeeded', 'HELLO_GESTURE': 'succeeded'}})

        if not text_pool:
            text_pool = ['Hello there!', 'Hello!', 'Hi!', 'Oh, hello!', 'olla!' 'allo!', 'Hey!']
        elif isinstance(str, text_pool):
            text_pool = [text_pool]
        hello_riding_gest = 'HR2I_simple_hello_gesture_wb'
        hello_standing_gest = 'HR2I_simple_hello_gesture_stand'
        _checkInstalledBehavior(hello_riding_gest)
        _checkInstalledBehavior(hello_standing_gest)

        with self:
            Concurrence.add('SAY_HELLO', SpeechFromPoolSM(pool=text_pool, blocking=True, wait_before_speak=2.95))

            hello_gest_sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'], input_keys=['riding_wifibot'])
            with hello_gest_sm:
                def choose_hello(ud):
                    if ud.riding_wifibot:
                        ud.hello_gest_name = hello_riding_gest
                        return 'on_wb'
                    ud.hello_gest_name = hello_standing_gest
                    return 'standing'
                StateMachine.add('CHOOSE_GESTURE', CBState(choose_hello, outcomes=['on_wb', 'standing'], input_keys=['riding_wifibot'],
                                                           output_keys=['hello_gest_name']),
                                 remapping={'hello_gest_name': 'behavior_name'},
                                 transitions={'on_wb': 'NAO_HELLO_GESTURE_WB', 'standing': 'NAO_HELLO_GESTURE_STAND'})
                StateMachine.add('NAO_HELLO_GESTURE_STAND', ExecuteBehavior(), transitions={'succeeded': 'succeeded'})
                StateMachine.add('NAO_HELLO_GESTURE_WB', ExecuteBehavior(), transitions={'succeeded': 'succeeded'})

            Concurrence.add('HELLO_GESTURE', hello_gest_sm)


class ReadObjectSegmentationTopic(ReadTopicState):
    def __init__(self, cluster_topic='/kinect2_clusters', timeout=5):
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
                    if ud.gesture_rec_result.gestureId == GestureRecognitionResult.idHello:
                        return 'hello_recognized'
                    elif ud.gesture_rec_result.gestureId == GestureRecognitionResult.idPointAt:
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
            StateMachine.add('SAY_NO_RECOGNITION', SpeechFromPoolSM(pool=text_pool, blocking=False),
                             transitions={'succeeded': 'WAIT_FOR_GESTURE', 'preempted': 'WAIT_FOR_GESTURE', 'aborted': 'WAIT_FOR_GESTURE'})


def _Pose2D_to_str(pose):
    # Small function to get a str from a Pose2D in a single line for displaying purposes.
    # Maybe should be moved to some better place so it can be used in more scripts
    if isinstance(pose, Pose2D):
        return '(' + str(pose.x) + ', ' + str(pose.y) + ', ' + str(pose.theta) + ')'
    return '(' + str(pose.x) + ', ' + str(pose.y) + ', ' + str(pose.z) + ')'


class WBMoveCloseToPoint(StateMachine):
    ''' Make the wifibot go to a location which is in the direction of a point but stopping at dist_to_loc m'''
    def __init__(self, dist_to_loc=0.2):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['ground_point'])

        with self:
            def prepare_req(ud):
                # Compute the ground_point pose but displaced dist_to_loc metres in the direction of the robot
                # To do so an equation system has been solved with the following equations (Assuming robot at 0,0):
                #   Line equation: y = (gy/gx) * x -- being (gx, gy) the ground point
                #   Pitagoras: k^2 = a^2 + b^2 -- being k = dist_to_loc   . (xg, yg)
                #   x = gx - a  --                                       /|
                #   y = gy - b  --                                      / | <- a
                #                                                      /__|
                #                                                (x,y).  ^b
                # (Resolution began substituing 4th equation to the first)
                #_a = math.sqrt((dist_to_loc**2) / (((-float(ud.in_ground_point.y)/ud.in_ground_point.x)**2) + 1))
                #_b = math.sqrt(dist_to_loc**2 - _a**2)

                ###### Easier mode using thales teorem
                ##         . (xg, yg)
                ##    k ->/|    -- being k = dist_to_loc
                ##(x, y)./_| <- a
                ##      /^b|                                  x
                ##     /   | <- A (whole) = xg                ^
                ##    /____|                             y <__|
                ##   .(0,0)^B = yg
                ##########
                # Euclidean dist between point 0 and ground_point (as robot reference point is 0,0)
                D = math.sqrt(ud.in_ground_point.x**2+ud.in_ground_point.y**2)
                if D <= dist_to_loc:
                    ud.out_move_req = MoveToServiceRequest()
                    return 'succeeded'
                _a = dist_to_loc*float(ud.in_ground_point.x)/D
                _b = dist_to_loc*float(ud.in_ground_point.y)/D
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
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'aborted'})


class NaoGoToLocationInFront(StateMachine):
    ''' Make the NAO go to a location which is in front of the object, and finish facing it
    (so robot and world coordinates are even)
    input parameters are the alpha which is the angle with respect to the start location and the point.
    Basically:       /
            x_world /   . (x_object, y_object) -- in NAO space
            ^      /    |
            |     /     |  <- K
            |    /      |
            |   /       . (x_object-K, y_object) -- in world space
       .    |  /
         .  | /
           .|/
<-y_world- NAO
            ^-NAO may not be looking in the direction of x_world but in the other axis
            The method to solve this is basically creating a vector v = (-K, 0) in world coordinates
            and rotate it alpha degrees to have the direction in robot coordinates. Finally translate the
            object point in the directiWBMoveCloseToPointWBMoveCloseToPointon of the rotated v vector.
    '''

    def __init__(self, K=0.2):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['alpha', 'location_point'])

        def rotate_point2D(pose, alpha):
            _pose = Pose2D()
            _pose.x = pose.x * math.cos(alpha) - pose.y*math.sin(alpha)
            _pose.y = pose.x * math.sin(alpha) + pose.y*math.cos(alpha)
            return _pose

        with self:
            def prepare_pose(ud):
                vec = Pose2D(-K, 0.0, 0.0)  # Translation vector in world space
                rot_v = rotate_point2D(vec, -ud.in_alpha)  # Rotated point. I don't know why - but... -
                translated_loc = Pose2D()
                translated_loc.x = ud.in_location_point.x + rot_v.x
                translated_loc.y = ud.in_location_point.y + rot_v.y
                translated_loc.theta = -ud.in_alpha  # Destination point will be with the alpha rotation corrected
                ud.out_new_loc = translated_loc
                rospy.loginfo('--- NaoGoToLocationInFront SM -- initial position: ' + _Pose2D_to_str(ud.in_location_point) +
                              ', translated_position' + _Pose2D_to_str(translated_loc))
                rospy.loginfo('--- NaoGoToLocationInFront SM -- translation vector = ' + str((vec.x, vec.y)) +
                              ', ' + str(ud.in_alpha) + 'rad rotated translation vector = ' + str((rot_v.x, rot_v.y)))
                return 'succeeded'

            StateMachine.add('PREPARE_POSE', CBState(prepare_pose, input_keys=['in_alpha', 'in_location_point'],
                                                     output_keys=['out_new_loc'], outcomes=['succeeded']),
                             remapping={'in_alpha': 'alpha', 'in_location_point': 'location_point', 'out_new_loc': 'target_loc'},
                             transitions={'succeeded': 'MOVE_TO_FRONT_OBJECT'})

            StateMachine.add('MOVE_TO_FRONT_OBJECT', MoveToState(), remapping={'objective': 'target_loc'},
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'preempted': 'aborted'})
