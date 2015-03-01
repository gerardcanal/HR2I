#!/usr/bin/env python
# Author: Gerard Canal Camprodon (gcanalcamprodon@gmail.com - github.com/gerardcanal)
from smach import Concurrence, StateMachine, CBState
from nao_smach_utils.get_user_speech_answer import GetUserAnswer
from hr2i_smach_states import WaitForGestureRecognitionSM
from nao_smach_utils.speech_recognition_states import StopRecognitionState
from nao_smach_utils.tts_state import SpeechFromPoolSM


class WaitForYesNoGestureSM(StateMachine):
    def __init__(self):
        StateMachine.__init__(outcomes=['succeeded'], output_keys=['user_answer'])

        with self:
            StateMachine.add('WAIT_FOR_GESTURE', WaitForGestureRecognitionSM(),
                             remapping={'out_ground_point': 'ground_point', 'out_person_position': 'person_position'},
                             transitions={'hello_recognized': 'SAY_HELLO', 'pointat_recognized': 'SAY_POINTAT',
                                          'nod_recognized': 'PREPARE_RESULT_YES', 'negate_recognized': 'PREPARE_RESULT_NO'})

            hello_pool = ['Oh, hi there! Can you answer yes or no?', 'Hi! Answer me now please.', 'Hi! Was it the one I told?']
            StateMachine.add('SAY_HELLO', SpeechFromPoolSM(pool=hello_pool),
                             transitions={'succeeded': 'WAIT_FOR_GESTURE'})
            point_pool = ['You pointed there but did not answer my question.', 'Can you answer my question?', 'Was it the one I told?']
            StateMachine.add('SAY_POINTAT', SpeechFromPoolSM(pool=point_pool),
                             transitions={'succeeded': 'WAIT_FOR_GESTURE'})

            def put_yes(ud):
                ud.user_answer = 'yes'
                return 'succeeded'
            StateMachine.add('PREPARE_RESULT_YES', CBState(put_yes, outcomes=['succeeded'], output_keys=['user_answer']),
                             remapping={'user_answer': 'user_answer'},
                             transitions={'succeeded': 'STOP_LISTENING'})

            def put_no(ud):
                ud.user_answer = 'no'
                return 'succeeded'
            StateMachine.add('PREPARE_RESULT_NO', CBState(put_no, outcomes=['succeeded'], output_keys=['user_answer']),
                             remapping={'user_answer': 'user_answer'},
                             transitions={'succeeded': 'STOP_LISTENING'})

            StateMachine.add('STOP_LISTENING',  # To ensure that the speech recognition will be stopped if the gesture is recognized
                             StopRecognitionState(),
                             transitions={'succeeded': 'succeeded'})


class GetYesNoFromUser(Concurrence):
    ''' Gets a yes/no answer from the user either from speech or gesture channels'''
    def __init__(self):
        def child_term_cb(outcome_map):
            # terminate all running states if LISTEN_USER finished with outcome 'succeeded'
            if outcome_map['LISTEN_USER'] == 'succeeded':
                return True

            # terminate all running states if GET_YESNO_GESTURE finished
            if outcome_map['GET_YESNO_GESTURE'] == 'succeeded':
                return True

            # in all other case, just keep running, don't terminate anything
            return False

        def out_cb(outcome_map):
            if outcome_map['LISTEN_USER'] == 'succeeded':
                return 'succeeded'
            elif outcome_map['GET_YESNO_GESTURE'] == 'succeeded':
                return 'succeeded'
            return 'aborted'

        Concurrence.__init__(['succeeded', 'preempted', 'aborted'], 'aborted', output_keys=['user_answer'],
                             child_termination_cb=child_term_cb, outcome_cb=out_cb)

        with self:
            Concurrence.add('LISTEN_USER', GetUserAnswer(get_one=True, ask_for_repetition=True),
                            remapping={'recognition_result': 'user_answer'})

            Concurrence.add('GET_YESNO_GESTURE', WaitForYesNoGestureSM(),
                            remapping={'user_answer': 'user_answer'})
