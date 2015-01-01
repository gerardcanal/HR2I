#!/usr/bin/env python
import rospy
import smach_ros

from smach import StateMachine
from nao_smach_utils.check_nodes import CheckNodesState
from nao_smach_utils.home_onoff import HomeOff_SM
from nao_smach_utils.stiffness_states import EnableStiffnessState
from nao_smach_utils.speech_recognition_states import SetSpeechVocabularyState
from hr2i_thesis.hr2i_mainSM import HR2I_SM

if __name__ == "__main__":
    rospy.init_node('HR2I_main_pipeline_node')

    TOPIC_LIST_NAMES = ['/recognized_gesture', '/kinect2_clusters', '/wifibot/odom']
    SERVICES_LIST_NAMES = ['/cmd_pose_srv', '/wb_move_to_srv']
    ACTION_LIST_NAMES = ['/speech', '/joint_angles_action']
    PARAMS_LIST_NAMES = []

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('CHECK_NODES', CheckNodesState(TOPIC_LIST_NAMES, SERVICES_LIST_NAMES, ACTION_LIST_NAMES, PARAMS_LIST_NAMES),
                         transitions={'succeeded': 'SET_VOCABULARY', 'aborted': 'aborted'})

        StateMachine.add('SET_VOCABULARY',
                         SetSpeechVocabularyState(['yes', 'no']),
                         transitions={'succeeded': 'HOME_ON'})

        StateMachine.add('HOME_ON', EnableStiffnessState(), transitions={'succeeded': 'HR2I_SM'})
        StateMachine.add('HR2I_SM', HR2I_SM(), transitions={'succeeded': 'HOME_OFF'})
        StateMachine.add('HOME_OFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})

    sis = smach_ros.IntrospectionServer('hr2i_iserver', sm, '/HR2I_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    print 'Press Ctrl+C to exit the node...'
    rospy.spin()
    sis.stop()
    print  # So the prompt is not concatenated to the ^C symbol in the command line
