@echo off
set DST_IP=onix
set FPS=30
: not working dunno why.. :screen-mouse-image=mousepointer.png

"c:\Program Files\VideoLAN\VLC\vlc.exe" screen:// :screen-fps=%FPS% :screen-caching=300 --sout "#transcode{vcodec=h264,vb=1000,scale=Automático,acodec=none}:rtp{dst=%DST_IP%,port=1234,mux=ts}"