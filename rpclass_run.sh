PS_NAME=rp
xhost +

docker stop $PS_NAME 2>/dev/null
docker rm $PS_NAME 2>/dev/null

docker run -it --gpus all --privileged \
-e DISPLAY=$DISPLAY \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-v /dev:/dev:rw \
-v /home/kmk/git/rp_nanosaur/src:/home/user/rp_nanosaur/src \
-e NVIDIA_DRIVER_CAPABILITIES=all \
-u user \
-w /home/user \
--hostname $(hostname) \
--group-add dialout \
--network host \
--shm-size 4096m \
--name $PS_NAME mdeagewt/rp_nanosaur:eloquent bash
