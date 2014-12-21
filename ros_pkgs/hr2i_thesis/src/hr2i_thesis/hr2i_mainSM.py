#!/usr/bin/env python
from smach import StateMachine
from nao_smach_utils.tts_state import SpeechFromPoolSM
from hr2i_smach_states import WaitForGestureRecognitionSM, NaoSayHello
from hr2i_pointat_sm import PointAtResponseExecutionSM


class HR2I_SM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.userdata.NAO_riding = True  # At the beginning, NAO will be riding the WifiBot

        with self:
            StateMachine.add('WAIT_FOR_GESTURE', WaitForGestureRecognitionSM(),
                             transitions={'hello_recognized': 'SAY_HELLO', 'pointat_recognized': 'POINT_AT_SM'},
                             remapping={'out_ground_point': 'ground_point'})

            StateMachine.add('SAY_HELLO', NaoSayHello(), transitions={'succeeded': 'WAIT_FOR_GESTURE'})

            StateMachine.add('POINT_AT_SM', PointAtResponseExecutionSM(),
                             transitions={'succeeded': 'SAY_DONE'})

            text_pool = ['I am done! What else?', 'No more gestures for me?', 'Look I got it!', 'Heyo I am awesome!']
            StateMachine.add('SAY_DONE', SpeechFromPoolSM(pool=text_pool), transitions={'succeeded': 'WaitForGestureRecognitionSM'})
