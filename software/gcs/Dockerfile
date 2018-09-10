FROM ros:melodic-robot

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /usr/src/
WORKDIR /usr/src/app/
ADD src /usr/src/app/src
ADD install_geographiclib_datasets.sh /usr/src/app/

RUN apt update && apt install -y ros-melodic-mavros ros-melodic-mavlink protobuf-compiler ros-melodic-mavros-extras ros-melodic-mavros-msgs python-pip python3-pip iptables

RUN wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.48.tar.gz && tar xfpz GeographicLib-1.48.tar.gz
RUN cd GeographicLib-1.48 && mkdir build && cd build && cmake .. && make -j4 && make install
RUN /bin/bash ./install_geographiclib_datasets.sh

RUN source /opt/ros/melodic/setup.bash && catkin_make -j4 || catkin_make -j4 || catkin_make -j4
ADD requirements.txt /usr/src/app/
ADD run_with_mavros.sh /usr/src/app/
RUN pip3 install future
RUN pip3 install -r requirements.txt
EXPOSE 14540

ENTRYPOINT ["./run_with_mavros.sh"]
