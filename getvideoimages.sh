mkdir video_file
cd video_file
rosrun image_view extract_images _sec_per_frame:=0.005 image:=/nao_robot/camera/top/camera/image_raw &
sleep 1
rosbag play /home/gerard/Escriptori/nao_test_2015-01-01-20-30-19.bag  # FIXME move to argument via cmd line
FPS=1
BITRATE=2400
WIDTH=640
HEIGHT=480
mencoder -nosound mf://*.jpg -mf w=$WIDTH:h=$HEIGHT:type=jpg:fps=$FPS -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=$BITRATE :mbd=2:keyint=132:v4mv:vqmin=3:lumi_mask=0.07:dark_mask=0.2:mpeg_quant:scplx_mask=0.1:tcplx_mask=0.1:naq -o ../nao_video.mp4
cd ..
rm -r video_file
