#!/usr/bin/expect -f

#sshpass -p "nao" scp -r nao@$NAO_IP:/home/nao/recordings/cameras/NAO_view_video.avi .
spawn scp -r nao@$env(NAO_IP):/home/nao/recordings/cameras/NAO_view_video.avi .
expect "Password: "
sleep 2 
exp_send "nao\r"
interact