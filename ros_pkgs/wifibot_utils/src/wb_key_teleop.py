#!/usr/bin/env python
import rospy
import curses
from wb_go_to_srv import WB_SPEED_TOPIC, speed_msg, WifiBotMoveToService

LINEAR_V = 0.5
ANGULAR_W = 0.5

if __name__ == '__main__':
    rospy.init_node('WB_KEYBOARD_TELEOP')
    pub = rospy.Publisher(WB_SPEED_TOPIC, speed_msg, queue_size=10)
    
    ## Keyboard handling
    stdscr = curses.initscr()
    #curses.cbreak()
    curses.noecho() 
    stdscr.keypad(1)

    stdscr.addstr(0,10,"Hit 'q' to quit")
    stdscr.refresh()

    key = ''
    while key != ord('q'):
        v = 0.0
        w = 0.0
        key = stdscr.getch()
        #stdscr.addch(20,25,key)
        stdscr.refresh()
        if key == curses.KEY_UP or key == ord('w'): 
            #stdscr.addstr(2, 20, "Up")
            v += LINEAR_V
        if key == curses.KEY_DOWN or key == ord('s'): 
            #stdscr.addstr(3, 20, "Down")
            v -= LINEAR_V
        if key == curses.KEY_LEFT or key == ord('a'):
            #stdscr.addstr(3, 0, "Left")
            w -= ANGULAR_W
        if key == curses.KEY_RIGHT or key == ord('d'):
            #stdscr.addstr(3, 30, "Right")
            w += ANGULAR_W
        (vl, vr) = WifiBotMoveToService.uni_to_diff(v, w)
        pub.publish(vl, vr)
        stdscr.addstr(3, 30, str(v)+'  '+str(w)+' ... '+str(vl)+' '+str(vr))

    curses.endwin()