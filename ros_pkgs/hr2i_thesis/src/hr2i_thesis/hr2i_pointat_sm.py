#!/usr/bin/env python
from smach import StateMachine, CBState
from hr2i_smach_states import ReadObjectSegmentationTopic, ReleaseNAOFromWifiBotState, WBMoveCloseToPoint
from nao_smach_utils.move_to_state import MoveToState

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

            StateMachine.add('MOVE_TO_POINTING_PLACE', WBMoveCloseToPoint(), transitions={'succeeded': 'WAIT_FOR_BLOBS'},
                             remapping={'request': 'move_to_request'})

            StateMachine.add('WAIT_FOR_BLOBS', ReadObjectSegmentationTopic(), transitions={'succeeded': 'RELEASE_NAO'})

            StateMachine.add('RELEASE_NAO', ReleaseNAOFromWifiBotState(), transitions={'succeeded': 'NAO_GO_TO_BLOB'},
                             remapping={'NAO_riding': 'out_NAO_riding'})

            StateMachine.add('NAO_GO_TO_BLOB', MoveToState(), remapping={'objective': '???FIXME'},
                             transitions={'succeeded': 'FIXME'})  # TODO
