#!/usr/bin/env python
from smach import StateMachine
from hr2i_thesis.msg import Kinect2Command
from nao_smach_utils.tts_state import SpeechFromPoolSM
from hr2i_smach_states import WaitForGestureRecognitionSM, NaoSayHello, SendCommandState
from hr2i_pointat_sm import PointAtResponseExecutionSM


class HR2I_SM(StateMachine):
    def __init__(self):
        StateMachine.__init__(self, outcomes=['succeeded', 'preempted', 'aborted'])
        self.userdata.NAO_riding_wb = True  # At the beginning, NAO will be riding the WifiBot

        with self:
            welcome_pool = ['Welcome! I am here to help you', 'Hey there! How may I help you?', 'Hello! I understand some gestures. Do you wanna try?']
            StateMachine.add('WELCOME', SpeechFromPoolSM(pool=welcome_pool), transitions={'succeeded': 'WAIT_FOR_GESTURE'})  # It begins recognizing gestures

            StateMachine.add('WAIT_FOR_GESTURE', WaitForGestureRecognitionSM(),
                             remapping={'out_ground_point': 'ground_point', 'out_person_position': 'person_position'},
                             transitions={'hello_recognized': 'SAY_HELLO', 'pointat_recognized': 'POINT_AT_SM'})

            StateMachine.add('SAY_HELLO', NaoSayHello(), remapping={'riding_wifibot': 'NAO_riding_wb'},
                             transitions={'succeeded': 'SEND_RECOGNIZE_GESTURE_CMD'})

            StateMachine.add('SEND_RECOGNIZE_GESTURE_CMD', SendCommandState(Kinect2Command.recGestCmd),
                             transitions={'succeeded': 'WAIT_FOR_GESTURE'})

            StateMachine.add('POINT_AT_SM', PointAtResponseExecutionSM(),
                             remapping={'in_ground_point': 'ground_point', 'in_NAO_riding': 'NAO_riding_wb', 'out_NAO_riding': 'NAO_riding_wb'},
                             transitions={'succeeded': 'SAY_DONE', 'not_riding_wb': 'SAY_CAN_NOT'})

            cannot_pool = ['I see you are pointing somewhere, but I can not go there as I am not riding the wifibot.',
                           'It would be cool to help you, but I am not on the wifibot.',
                           'I can not get there without my cool wifibot transport.',
                           'I would wish to go there, but I am not on the wifibot and too tired to walk there',
                           'I am tired and I will not walk to that position. I am sorry']
            StateMachine.add('SAY_CAN_NOT', SpeechFromPoolSM(pool=cannot_pool),
                             transitions={'succeeded': 'SEND_RECOGNIZE_GESTURE_CMD', 'preempted': 'SEND_RECOGNIZE_GESTURE_CMD',
                                          'aborted': 'SEND_RECOGNIZE_GESTURE_CMD'})

            text_pool = ['I am done! What else?', 'No more gestures for me?', 'Look I got it!', 'Heyo I am awesome!']
            StateMachine.add('SAY_DONE', SpeechFromPoolSM(pool=text_pool),
                             transitions={'succeeded': 'SEND_RECOGNIZE_GESTURE_CMD', 'preempted': 'SEND_RECOGNIZE_GESTURE_CMD',
                                          'aborted': 'SEND_RECOGNIZE_GESTURE_CMD'})
