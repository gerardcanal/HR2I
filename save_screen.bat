@echo off
set DST_IP=onix
set FPS=30
: not working dunno why.. :screen-mouse-image=mousepointer.png

"c:\Program Files\VideoLAN\VLC\vlc.exe" screen:// :screen-fps=%FPS% --sout "#transcode{vcodec=h264,vb=1000,scale=Automático,acodec=none}:std{mux=mp4,access=file,dst=C:\Users\upcnet\Desktop\gerard.canal\screencast.mp4}"