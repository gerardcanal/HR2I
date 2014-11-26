#!/usr/bin/env python
import rospy
import curses
import math
from threading import Thread
from wb_go_to_srv import WB_SPEED_TOPIC, speed_msg, WifiBotMoveToService

LINEAR_V = 0.02
ANGULAR_W = 0.02
HZ = 10

class KeyTeleop:
    def __init__(self, interface):
        self._interface = interface
        self._pub = rospy.Publisher(WB_SPEED_TOPIC, speed_msg, queue_size=1)

        curses.cbreak()
        curses.noecho() 
        self._interface.keypad(1)
        self._interface.nodelay(True)

        self._interface.addstr(0,10,"Hit 'Ctrl+C' to quit")
        self._interface.refresh()

    def start(self):
        self._stop_requested = False
        self._thread = Thread(target=self.keyloop)
        self._thread.start()

    def stop(self):
        self._stop_requested = True
        self._thread.join()

    def keyloop(self):
        key = ''
        v = 0
        w = 0
        rate = rospy.Rate(HZ)
        while not self._stop_requested:
            self._interface.refresh()
            key = self._interface.getch()
            if key == ord(' '):
                v = 0
                w = 0
            else:
                (v_, w_) = self.getVels(key)
                # v = min(abs(v+v_), LINEAR_V)*math.copysign(1, v+v_)
                # w = min(abs(w+w_), ANGULAR_W)*math.copysign(1, w+w_)
                v = v+v_
                w = w+w_
            (vl, vr) = WifiBotMoveToService.uni_to_diff(v, w)
            self._pub.publish(vl, vr)
            self._interface.addstr(3, 30, str(v)+'  '+str(w))
            rate.sleep()

    def getVels(self, key):
        v = 0.0
        w = 0.0

        if key == curses.KEY_UP or key == ord('w'): 
            v += LINEAR_V
        if key == curses.KEY_DOWN or key == ord('s'): 
            v -= LINEAR_V
        if key == curses.KEY_LEFT or key == ord('a'):
            w += ANGULAR_W
        if key == curses.KEY_RIGHT or key == ord('d'):
            w -= ANGULAR_W

        return (v, w)

def main(stdscr):
    rospy.init_node('WB_KEYBOARD_TELEOP')
    app = KeyTeleop(curses.initscr())
    app.start()
    rospy.spin()
    app.stop()
    curses.endwin()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
        
        