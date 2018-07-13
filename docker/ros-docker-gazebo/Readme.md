Install docker https://docs.docker.com/engine/installation/linux/ubuntu/

To run docker without super user:

  sudo groupadd docker
  sudo gpasswd -a ${USER} docker
  sudo service docker restart


docker build -t ros-kinetic-gazebo7 .

xhost +
docker run \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/dri:/dev/dri \
  --env="DISPLAY" \
  your_image
