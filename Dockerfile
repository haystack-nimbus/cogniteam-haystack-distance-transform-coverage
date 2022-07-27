FROM ros:melodic

COPY ./map_coverage cogniteam_converage_exploration_ws/src/map_coverage


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]

RUN apt-get update && apt-get install ros-melodic-angles -y && \
    apt-get install ros-melodic-image-geometry -y && \
    apt-get install ros-melodic-tf2 -y  && apt-get install ros-melodic-tf -y && \
    apt-get install ros-melodic-image-transport-plugins -y && \
    apt-get install ros-melodic-image-transport -y && \
    apt-get install ros-melodic-move-base -y && \
    apt-get install ros-melodic-move-base-msgs -y && \
    apt-get install ros-melodic-actionlib-msgs && \
    apt-get install ros-melodic-cv-bridge -y && rm /var/lib/apt/lists/* -rf
    
WORKDIR /cogniteam_converage_exploration_ws/

RUN . /opt/ros/melodic/setup.sh && catkin_make


#sudo docker buildx build --platform linux/amd64,linux/arm64 -t cognimbus/cogniteam-mce:latest --push .

