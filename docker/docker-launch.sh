#!/bin/bash

container_name=$1
image_name=$2
docker_username=$3

XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi
xhost +local:docker

# ============================================ #
#           SETTING UP USER WORKSPACE          #
# ============================================ #

# timestamp=`date +"%Y%m%d_%H%M%S"`

if [[ ! -d ~/Docker/${container_name}_workspace ]]; then
    echo "Creating directory ${container_name}_workspace /home/$USER/Docker/${container_name}_workspace"
    mkdir -p ~/Docker/${container_name}_workspace
fi


# ============================================ #
#           RUNNING DOCKER CONTAINER           #
# ============================================ #

ws_dir=/home/${docker_username}/workspace

docker run \
    -it \
    -e "TERM=xterm-256color" \
    --env="DISPLAY=$DISPLAY" \
    --net=host \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume="$XAUTH:$XAUTH" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    -v /home/$USER/Docker/${container_name}_workspace:/${ws_dir}:rw \
    --name "$container_name" "$image_name"
    # --gpus all \
    # --runtime=nvidia \
    # --mount type=bind,source=/home/$USER/Docker/${container_name}_workspace,target=${ws_dir} \

    # -v ${docker_username}:/${ws_dir}:rw 
