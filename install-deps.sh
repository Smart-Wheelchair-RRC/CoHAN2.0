#!/bin/bash 

rosdep update --include-eol

if [ -n "${ROS_DISTRO}" ];
then
	rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y
else
	echo "ROS_DISTRO not set!"
fi

# sudo apt-get install ros-melodic-pr2-description -y \
#                      libqt5widgets5 \
#                      ros-melodic-mbf-costmap-core \
#                      libqt5core5a \
#                      ros-melodic-costmap-converter \
#                      libzmq3-dev \
#                      qtbase5-dev \
#                      libsdl-image1.2-dev \
#                      libqt5gui5 \
#                      libsqlite3-dev \
#                      libyaml-cpp-dev \
#                      ros-melodic-libg2o \
#                      ros-melodic-pr2-tuckarm \
#                      libboost-all-dev \
#                      ros-melodic-mbf-msgs
