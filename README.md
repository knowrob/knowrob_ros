# knowrob_ros

**knowrob_ros** is a ROS 1 wrapper for [KnowRob](https://github.com/knowrob/knowrob), providing ROS interfaces to integrate KnowRob's knowledge reasoning capabilities with robotic systems. 

## Key Features

- **ROS Actions**  
  - `AskOne`, `AskAll`, **AskIncremental**, and `Tell`  
- **Python API**  
  - `knowrob_ros_lib.py` for simple query/assertion via ros in python
- **Docker Support**  
  - Ready-to-use container for fast setup  

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
git clone https://github.com/knowrob/knowrob_ros.git
```

### 2. Build the workspace

```bash
cd ~/catkin_ws
catkin build
```
---

## Installation with Docker

If you'd like to avoid setting up everything locally, you can use the provided Dockerfile to run KnowRob and knowrob_ros in an isolated container.

### 1. Build the Docker Image

From the root of the repository:

```bash
docker build -t knowrob/ros1 .
```

### 2. Run the Container

```bash
docker run -it --entrypoint bash knowrob/ros1
```

### 3. Launch KnowRob in Docker

Once inside the container:

```bash
source /catkin_ws/devel/setup.bash
roslaunch knowrob_ros knowrob.launch
```

## Usage

### ROS Interfaces

The following ROS interfaces are available:

| Endpoint                                       | Type                                 | Description                   |
|------------------------------------------------|--------------------------------------|-------------------------------|
| `/knowrob/askone`                              | `AskOneAction`                       | Single-result query           |
| `/knowrob/askall`                              | `AskAllAction`                       | Multi-result query            |
| `/knowrob/askincremental`                      | `AskIncrementalAction`               | Start incremental query       |
| `/knowrob/askincremental_next_solution`        | `AskIncrementalNextSolutionAction`   | Fetch next solution           |
| `/knowrob/askincremental_finish` (service)     | `AskIncrementalFinish.srv`           | Finish incremental query      |
| `/knowrob/tell`                                | `TellAction`                         | Assert triples into KB        |

All endpoints need to be called with a `ModalFrame` message. We document the `ModalFrame` message in detail below.

### Using the Interface from Python

Use the Python client library to integrate KnowRob queries and assertions directly into your code. Here’s a minimal example:

```python
from knowrob_ros.knowrob_ros_lib import KnowRobRosLib, get_default_modalframe

# Initialize ROS node and client
client = KnowRobRosLib()
client.init_node("my_python_client")

# Single-result query
modal = get_default_modalframe()
response = client.ask_one("lpn:jealous(lpn:vincent, X)", modal)
print("AskOne status:", response.status)
print("Binding:", response.answer)

# Shutdown when done
client.shutdown_node()
```

For detailed usage, full method reference, and advanced examples, see the [Python API Guide](src/knowrob_ros_lib/README.md).


### Command Line

To run a query insterface on the terminal (for docker you can connect to the same container in which you launched knowrob), you should first list to the corresponding topic, on which the results are published:

```bash
rostopic echo /knowrob/askone/result 
```

Then run a query on another terminal:

```bash
rostopic pub /knowrob/askone/goal knowrob_ros/AskOneActionGoal "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
goal_id:
  stamp: {secs: 0, nsecs: 0}
  id: ''
goal:
  query:
    lang: ''
    queryString: 'lpn:jealous(lpn:vincent, X)'
    frame:
      epistemicOperator: 0                                                                                                          
      aboutAgentIRI: ''
      aboutSimulationIRI: ''                                                             
      temporalOperator: 0
      minPastTimestamp: 0.0
      maxPastTimestamp: 0.0
      confidence: 0.0"
```

You should now see an answer on the first terminal.

## Modal Frames

The `ModalFrame` message controls **who** “knows” what, and **when** it’s considered true. Supply one on every call to `ask_one`, `ask_all`, `ask_incremental`, or `tell`.

| Field                 | Type     | Default                          | Description                                                                                                                       |
|-----------------------|----------|----------------------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `epistemicOperator`   | `uint8`  | `KNOWLEDGE (0)`                  | **KNOWLEDGE:** “it is certain that …”<br>**BELIEF (1):** “it could be that …”                                                    |
| `aboutAgentIRI`       | `string` | `""`                           | IRI of another agent whose knowledge/belief you’re querying. Mutually exclusive with `aboutSimulationIRI`.                      |
| `aboutSimulationIRI`  | `string` | `""`                           | IRI of a mental simulation knowledge graph to query. Mutually exclusive with `aboutAgentIRI`.                                     |
| `temporalOperator`    | `uint8`  | `CURRENTLY (0)`                  | **CURRENTLY:** “it is now the case that …”<br>**ALL_PAST (1):** “it always was the case that …”<br>**SOME_PAST (2):** “it was the case that …” |
| `minPastTimestamp`    | `float`  | `UNSPECIFIED_TIMESTAMP (-1.0)`   | Lower time bound (seconds since epoch). When >0, only facts true *after* this timestamp are considered.                          |
| `maxPastTimestamp`    | `float`  | `UNSPECIFIED_TIMESTAMP (-1.0)`   | Upper time bound (seconds since epoch). When >0, only facts true *before* this timestamp are considered.                         |
| `confidence`          | `float`  | `0.0`                            | Minimum confidence threshold (0.0–1.0) for returned statements.                                                                   |

## Repository Layout

```
.
├── action/            ← ROS action definitions
├── include/           ← C++ interface headers
├── launch/            ← Launch files
├── msg/               ← Custom ROS messages
├── scripts/           ← Python test scripts
├── src/               ← Core Prolog, C++ and Python code
│   └── knowrob_ros_lib/ ← Python client library
├── srv/               ← ROS services
├── test/              ← rostest definitions
├── Dockerfile         ← Container setup
├── build-docker.sh    ← Helper for Docker builds
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Contributing

1. Fork the repo and create a branch  
2. Write or update tests in `scripts/` or `test/`  
3. Follow ROS and Python style guidelines  
4. Submit a pull request  

## Support

- **Issues & feature requests**: [GitHub Issues](https://github.com/your-org/knowrob_ros/issues)  

(Inactive issues will be closed after 2 weeks of inactivity. Please open a new issue if you need further assistance.)
