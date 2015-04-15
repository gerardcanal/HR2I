#! /usr/bin/env python
# Based on REEM's Robocup 2014 GPSR test.
import rospy
import roslib
import sys
import time
from os.path import expanduser


PATH_TO_SOAR = expanduser('~/catkin_ws/src/soar/soar9.4.0x64/bin/')
sys.path.append(PATH_TO_SOAR)
SOAR_GP_PATH = str(roslib.packages.get_pkg_dir("hr2i_soar") + "/SOAR_project/hr2i.soar")
import Python_sml_ClientInterface as sml


### SOAR UTILS
def create_kernel():
    kernel = sml.Kernel.CreateKernelInCurrentThread()
    if not kernel or kernel.HadError():
        print kernel.GetLastErrorDescription()
        exit(-1)
    return kernel


def create_agent(kernel, name):
    agent = kernel.CreateAgent("agent")
    if not agent:
        print kernel.GetLastErrorDescription()
        exit(-1)
    return agent


def agent_load_productions(agent, path):
    agent.LoadProductions(path)
    if agent.HadError():
        print agent.GetLastErrorDescription()
        exit(-1)


### SKILLS
def call_disambiguate():
    rospy.loginfo("DISAMBIGUATING...")
    return 'succeeded'


def call_nao_approach_obj():
    rospy.loginfo('NAO APPROACHES TO THE OBJECT...')
    return 'succeeded'


def call_nao_go_down():
    rospy.loginfo('NAO GOES DOWN THE WIFIBOT...')
    return 'succeeded'


def call_nao_point_obj():
    rospy.loginfo('NAO POINTS THE OBJECT...')
    return 'succeeded'


def call_say_hello_riding():
    rospy.loginfo('NAO SAYS HELLO WHILE ON THE WIFIBOT...')
    return 'succeeded'


def call_say_hello_standing():
    rospy.loginfo('NAO SAYS HELLO WHILE STANDING UP...')
    return 'succeeded'


def call_segment_objects():
    rospy.loginfo('SEGMENTING OBJECTS FROM KINECT SENSOR...')
    return 'succeeded'


def call_wb_approach_loc():
    rospy.loginfo('WIFIBOT APPROACHES THE LOCATION...')
    return 'succeeded'


## MAIN
def execute_soar():
    first_time = time.time()
    kernel = create_kernel()
    agent = create_agent(kernel, "agent")
    agent_load_productions(agent, SOAR_GP_PATH)
    agent.SpawnDebugger()

    # p_cmd = 'learn --on'
    # res = agent.ExecuteCommandLine(p_cmd)
    # res = kernel.ExecuteCommandLine(p_cmd, agent.GetAgentName)
    kernel.CheckForIncomingCommands()
    p_cmd = 'watch --learning 2'
    res = agent.ExecuteCommandLine(p_cmd)
    print "Result:", str(res)

    goal_achieved = False
    while not goal_achieved:
        agent.Commit()
        agent.RunSelfTilOutput()
        agent.Commands()
        numberCommands = agent.GetNumberCommands()
        rospy.loginfo("Number of commands received from the agent: %s" % (numberCommands))
        i = 0
        if numberCommands == 0:
            rospy.logerr('Some error happened')
            return 'aborted'
        else:
            while i < numberCommands:
                command = agent.GetCommand(i)
                command_name = command.GetCommandName()
                print command, command_name
                print command.GetParameterValue("a")
                rospy.loginfo("The name of the %d/%d command is %s" % (i+1, numberCommands, command_name))
                if (time.time()-first_time) > 270:
                    rospy.logwarn('Timeout!')
                    return "succeeded"

                out = "NULL"
                if command_name == "disambiguate-objects":
                    #loc_to_navigate = command.GetParameterValue("loc")
                    out = call_disambiguate()

                elif command_name == "nao-approach-object":
                    out = call_nao_approach_obj()
                elif command_name == "nao-go-down-wb":
                    out = call_nao_go_down()
                elif command_name == "nao-point-obj":
                    out = call_nao_point_obj()
                elif command_name == "say-hello-riding":
                    out = call_say_hello_riding()
                elif command_name == "say-hello-standing":
                    out = call_say_hello_standing()
                elif command_name == "segment-objects":
                    out = call_segment_objects()
                elif command_name == "wb-approach-location":
                    out = call_wb_approach_loc()
                elif command_name == "achieved":
                    goal_achieved = True
                    #call_go_to('referee', world)
                    out = "succeeded"
                else:
                    rospy.logerr("ERROR: Command %s does not exist" % (command_name))
                    command.AddStatusComplete()

                rospy.loginfo("SM outcome was: %s \n\n" % (out))
                if out == "succeeded":
                    command.AddStatusComplete()
                elif out == "aborted":
                    command.AddStatusError()
                else:
                    rospy.logerr("hr2i_soar interface: unknown ERROR")
                    exit(-1)

                i += 1
                print i

    command.AddStatusComplete()
    return 'succeeded'

    kernel.DestroyAgent(agent)
    kernel.Shutdown()


if __name__ == "__main__":
    rospy.init_node("HR2I_SOAR_node")
    rospy.loginfo('Path to SOAR bin is: ' + PATH_TO_SOAR)
    #readtopic
    #creategoal
    execute_soar()
