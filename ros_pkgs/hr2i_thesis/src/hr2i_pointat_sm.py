#!/usr/bin/env python
from smach import StateMachine, CBState
from wifibot_smach.wifibot_goto_state import GoToPositionState
from wifibot_utils.srv import MoveToServiceRequest
from geometry_msgs.msg import Pose2D
from hr2i_smach_states import WaitForObjectBlobsState, ReleaseNAOFromWifiBotState

LAST_ORIENT = True
class PointAtResponseExecutionSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['in_ground_point'])
        with self:
            def prepare_gpoint(userdata): # FIXME may need to change x for z or y for z...
                # TODO subtract X from the position
                userdata.move_to_request = MoveToServiceRequest(Pose2D(userdata.in_ground_point.x, userdata._in_ground_point.y, 0.0), LAST_ORIENT)
                return 'succeeded'

            StateMachine.add('PREPARE_GROUND_POINT_OBJ', CBState(prepare_gpoint, 
                                                         input_keys=['in_ground_point'], output_keys=['move_to_request'], outcomes=['succeeded']),
                             transitions={'succeeded': 'MOVE_TO_POINTING_PLACE'})
            StateMachine.add('MOVE_TO_POINTING_PLACE', GoToPositionState(), transitions={'succeeded': 'WAIT_FOR_BLOBS'}, remapping={'request': 'move_to_request'})

            StateMachine.add('WAIT_FOR_BLOBS', WaitForObjectBlobsState(), transitions={'succeeded': 'RELEASE_NAO'})

            StateMachine.add('RELEASE_NAO', ReleaseNAOFromWifiBotState(), transitions={'succeeded': 'NAO_GO_TO_BLOB'})

            StateMachine.add('NAO_GO_TO_BLOB')