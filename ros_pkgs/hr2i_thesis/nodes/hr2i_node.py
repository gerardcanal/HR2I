#!/usr/bin/env python
import rospy
import smach_ros

from smach import StateMachine
from nao_smach_utils.check_nodes import CheckNodesState
from nao_smach_utils.home_onoff import HomeOff_SM
from nao_smach_utils.stiffness_states import EnableStiffnessState
from hr2i_mainSM import HR2I_SM

if __name__ == "__main__":
    rospy.init_node('HR2I_main_pipeline_node')

    TOPIC_LIST_NAMES = ['/recognized_gesture', '/kinect2_blobs']
    SERVICES_LIST_NAMES = ['/cmd_pose_srv', '/wb_move_to_srv']
    ACTION_LIST_NAMES = ['/speech', '/joint_angles_action']
    PARAMS_LIST_NAMES = []

    # Create a SMACH state machine
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.add('CHECK_NODES', CheckNodesState(TOPIC_LIST_NAMES, SERVICES_LIST_NAMES, ACTION_LIST_NAMES, PARAMS_LIST_NAMES),
                         transitions={'succeeded': 'HOME_ON', 'aborted': 'aborted'})

        StateMachine.add('HOME_ON', EnableStiffnessState(), transitions={'succeeded': 'HR2I_SM'})  # FIXME maybe just stiffen some joints?
        StateMachine.add('HR2I_SM', HR2I_SM(), transitions={'succeeded': 'HOME_OFF'})
        StateMachine.add('HOME_OFF', HomeOff_SM(), transitions={'succeeded': 'succeeded'})

    sis = smach_ros.IntrospectionServer('hr2i_iserver', sm, '/HR2I_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
