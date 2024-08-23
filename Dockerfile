FROM ros:noetic
MAINTAINER Sascha Jongebloed, jongebloed@uni-bremen.de

ENV SWI_HOME_DIR=/usr/lib/swi-prolog
ENV LD_LIBRARY_PATH=/usr/lib/swi-prolog/lib/x86_64-linux:$LD_LIBRARY_PATH

RUN apt update
RUN apt install -y pip libeigen3-dev libspdlog-dev \
    libraptor2-dev mongodb-clients \
    libmongoc-1.0-0 libmongoc-dev \
    libfmt-dev software-properties-common \
    git python3-catkin-pkg python3-catkin-tools \
    libboost-all-dev librdf0-dev

RUN apt-add-repository ppa:swi-prolog/stable
RUN apt update
RUN apt install -y swi-prolog*

RUN pip install -U rosdep future

# Create user 'ros'
RUN useradd -m -d /home/ros -p ros ros && \
    adduser ros sudo && \
    chsh -s /bin/bash ros
ENV HOME /home/ros
WORKDIR /home/ros

# Switch to the new user 'ros'
USER ros
RUN mkdir /home/ros/src && \
    chown -R ros:ros /home/ros && \
    rosdep update

RUN echo "source /opt/ros/noetic/setup.bash" >> /home/ros/.bashrc && \
    echo "source /home/ros/.bashrc" >> /home/ros/.bash_profile

ENV PATH=/home/ros/devel/bin:/opt/ros/noetic/bin:.:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games
ENV ROS_PACKAGE_PATH=/home/ros/src:/opt/ros/noetic/share:/opt/ros/noetic/stacks
ENV CMAKE_PREFIX_PATH=/home/ros/noetic/catkin_ws/devel:/opt/ros/noetic
ENV PKG_CONFIG_PATH=/home/ros/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_WORKSPACE=/home/ros
ENV ROS_IP=127.0.0.1
ENV SWI_HOME_DIR=/usr/lib/swi-prolog
#ENV PYTHONPATH=/home/ros/devel/lib/python2.7/dist-packages:/opt/ros/noetic/lib/python2.7/dist-packages
ENV LD_LIBRARY_PATH=/home/ros/devel/lib:/opt/ros/melodic/lib:/opt/ros/noetic/lib/python2.7/dist-packages

# Forward ports: webserver + rosbridge
EXPOSE 1111 9090

# Initialize the catkin workspace
USER ros
WORKDIR /home/ros/src
#RUN /opt/ros/noetic/bin/catkin_init_workspace -DPYTHON_EXECUTABLE=/usr/bin/python3
RUN /usr/bin/catkin init

# Clone the knowrob repository
RUN git clone https://github.com/knowrob/knowrob.git --branch ros
# Add the knowrob-ros1 repository
ADD . /home/ros/src/knowrob_ros

WORKDIR /home/ros
# Build the catkin workspace
RUN /usr/bin/catkin build --verbose

COPY run_knowrob.sh /run_knowrob.sh
COPY run_knowrob_local.sh /run_knowrob_local.sh

ENTRYPOINT ["/run_knowrob.sh"]
