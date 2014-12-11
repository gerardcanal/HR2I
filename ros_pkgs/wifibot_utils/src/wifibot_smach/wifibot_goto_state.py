#!/usr/bin/env python
from smach_ros import ServiceState
from wifibot_utils.srv import MoveToService, MoveToServiceRequest
from geometry_msgs.msg import Pose2D


class GoToPositionState(ServiceState):

    def __init__(self, request=None):
        ''' request should be a MoveToServiceRequest or a tuple with 4 elements (x,y,theta,orient). The same if it is passed by userdata '''
        # Private attributes
        input_keys = []
        if not request:
            input_keys = ['request']
            self._request = None
        elif not isinstance(request, MoveToServiceRequest):
            if len(request) != 4:
                raise Exception('Tuple passed to GoToPositionState was not of 4 elements. Argument should be a MoveToServiceRequest or a tuple with (x,y,theta,orient).')
            self._request = MoveToServiceRequest(Pose2D(request[0], request[1], request[2]), request[3])
        else:
            self._request = request

        # Class constructor
        ServiceState.__init__(self, '/wb_move_to_srv', MoveToService, outcomes=['succeeded'], request_cb=self.go_to_request_cb, input_keys=input_keys)

    def go_to_request_cb(self, ud, request):
        if (not self._request):
            if not isinstance(request, MoveToServiceRequest):
                if len(request) != 4:
                    raise Exception('Tuple passed to GoToPositionState was not of 4 elements. Argument should be a MoveToServiceRequest or a tuple with (x,y,theta,orient).')
                self._request = MoveToServiceRequest(Pose2D(ud.request[0], ud.request[1], ud.request[2]), ud.request[3])
            else:
                self._request = ud.request
        return self._request
