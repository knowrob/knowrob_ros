# knowrob_ros

**knowrob_ros** is a ROS 1 wrapper for [KnowRob](https://github.com/knowrob/knowrob), providing ROS interfaces to integrate KnowRob's knowledge reasoning capabilities with robotic systems. This package bridges KnowRob's Prolog-based system with common ROS workflows, making it easier to perform semantic reasoning in real-world robotic tasks.

## Installation (Local ROS Workspace)

### Prerequisites

- Ubuntu 20.04
- ROS Noetic

For the rest, see the dependencies of [KnowRob](https://github.com/knowrob/knowrob).

### 1. Clone the Repositories

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone KnowRob
git clone https://github.com/knowrob/knowrob.git

# Clone knowrob_ros (this repo)
git clone https://github.com/your-org/knowrob_ros.git
```

### 2. Build the workspace

```bash
cd ~/catkin_ws
catkin build
```
---

## 🐳 Installation with Docker

If you'd like to avoid setting up everything locally, you can use the provided Dockerfile to run KnowRob and knowrob_ros in an isolated container.

### 1. Build the Docker Image

From the root of the repository:

```bash
docker build -t knowrob_ros .
```

### 2. Run the Container

```bash
docker run -it --entrypoint bash knowrob_ros
```

### 3. Launch KnowRob in Docker

Once inside the container:

```bash
source /catkin_ws/devel/setup.bash
roslaunch knowrob_ros knowrob.launch
```