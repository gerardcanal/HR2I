mkdir video_file
cd video_file
rosrun myimage_view extract_images _sec_per_frame:=0.002 image:=/nao_robot/camera/top/camera/image_raw &
sleep 1
rosbag play $1
kill %
sleep 1
FPS=1
BITRATE=2400
WIDTH=640
HEIGHT=480
mencoder -nosound mf://*.jpg -mf w=$WIDTH:h=$HEIGHT:type=jpg:fps=$FPS -ovc copy -o $1.mp4
#mencoder -nosound mf://*.jpg -mf w=$WIDTH:h=$HEIGHT:type=jpg:fps=$FPS -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=$BITRATE:mbd=2:keyint=132:v4mv:vqmin=3:lumi_mask=0.07:dark_mask=0.2:mpeg_quant:scplx_mask=0.1:tcplx_mask=0.1:naq -o $1.mp4
cd ..
rm -r video_file
