# knowrob_ros Python Client Library

`knowrob_ros_lib.py` provides a lightweight interface to KnowRob's ROS action and service endpoints. 

## Quickstart

A minimal end-to-end example that:
1. Initializes ROS and the client
2. Performs an AskOne query
3. Shuts down cleanly

```python
from knowrob_ros.knowrob_ros_lib import KnowRobRosLib, get_default_modalframe

# 1) Initialize
client = KnowRobRosLib()
client.init_node("quickstart_client")

# 2) Query for "lpn:jealous(lpn:vincent, X)"
modal = get_default_modalframe()
res = client.ask_one("lpn:jealous(lpn:vincent, X)", modal)

print(f"Status: {res.status}\nBinding: {res.answer}")

# 3) Shutdown
client.shutdown_node()
```

For more detailed examples (AskAll, incremental queries, assertions), see the [Python API Guide](../src/knowrob_ros_lib/README.md).

---

## API Reference

### `class KnowRobRosLib`

High-level client exposing all KnowRob ROS endpoints:

| Method                                         | Description                                                      |
|------------------------------------------------|------------------------------------------------------------------|
| `init_node(name: str)`                         | Initialize ROS node, action clients, and service proxies.        |
| `shutdown_node()`                              | Shutdown ROS node and cancel any pending goals.                  |
| `ask_one(query: str, frame: ModalFrame, lang)` | Single-result query; returns `AskOneResult`.                     |
| `ask_all(query: str, frame: ModalFrame, lang)` | Multi-result query; returns `AskAllResult`.                      |
| `ask_incremental(query: str, frame: ModalFrame)` | Start incremental query; returns `AskIncrementalResult`.       |
| `next_solution(query_id: int)`                 | Get next solution for incremental query; returns `AskIncrementalNextSolutionResult`. |
| `finish_incremental(query_id: int)`            | Finish incremental query; returns `bool` success indicator.      |
| `tell(triples: list[Triple], frame: ModalFrame)` | Assert a list of RDF triples; returns `TellResult`.           |


## Tutorial

This tutorial walks through common workflows step-by-step, from basic queries to advanced incremental patterns.

### 1. Basic Single-Result Query

```python
# Import client and modal frame utility
from knowrob_ros.knowrob_ros_lib import KnowRobRosLib, get_default_modalframe,
binding = graph_answer_to_dict


# Create the client instance
client = KnowRobRosLib()  # Instantiate the KnowRob ROS client

# Initialize the ROS node named 'tutorial_client'
client.init_node("tutorial_client")  # Must be called before any ROS actions

# Obtain a default modal frame for contextual queries
modal = get_default_modalframe()  # Sets up epistemic & temporal defaults

# Execute a single-result query (AskOne)
result = client.ask_one(
    "lpn:jealous(lpn:vincent, X)",  # Prolog query string
    modal                            # ModalFrame argument
)

# Check the query status
if result.status == AskOneResult.TRUE:  # TRUE indicates a successful match
    # Convert the raw GraphAnswerMessage to a Python dict
    binding = graph_answer_to_dict(result.answer)
    # Print the bound value for X
    print("Jealous of:", binding['X'])
else:
    # Handle cases where no result is found or an error occurred
    print("No result or query failed")

# Clean shutdown of the ROS client
client.shutdown_node()  # Gracefully closes connections and cancels goals
```

### 2. Retrieving All Matches

```python
from knowrob_ros.knowrob_ros_lib import KnowRobRosLib, get_default_modalframe, graph_answers_to_list

# 1) Initialize client and node
client = KnowRobRosLib()                          # Instantiate client
client.init_node("tutorial_client")             # Initialize ROS node
modal = get_default_modalframe()                  # Default modal frame

# 2) Execute a multi-result query (AskAll)
res_all = client.ask_all(
    "childOf(X, lpn:vincent)",  # Query to find all children of Vincent
    modal                         # ModalFrame argument
)

# 3) Convert list of GraphAnswerMessage to Python list of dicts
bindings = graph_answers_to_list(res_all.answers)  # [{ 'X': ... }, ...]

# 4) Iterate and print each binding
for b in bindings:
    print(b)  # Print each result dictionary

# 5) Shutdown
client.shutdown_node()  # Graceful shutdown
```

### 3. Incremental Queries

```python
from knowrob_ros.knowrob_ros_lib import (
    KnowRobRosLib,
    get_default_modalframe,
    graph_answer_to_dict
)

# 1) Initialize
client = KnowRobRosLib()                          # Instantiate client
client.init_node("tutorial_client")             # Initialize ROS node
modal = get_default_modalframe()                  # Default modal frame

# 2) Start incremental query (AskIncremental)
inc = client.ask_incremental(
    "relatedTo(X, Y)",  # Query string for relation pairs
    modal                 # ModalFrame
)
# The server returns a queryId to fetch solutions one by one
query_id = inc.queryId    # Unique identifier for this incremental session

# 3) Loop until no more solutions
while True:
    sol = client.next_solution(query_id)        # Fetch next solution
    if sol.finished:                            # Check if iteration is complete
        break
    # Convert and print the answer
    binding = graph_answer_to_dict(sol.answer)  # Extract variable bindings
    print(binding)                              # Print the dict, e.g. { 'X': ..., 'Y': ... }

# 4) Release resources on the server
success = client.finish_incremental(query_id)  # Invoke service to finish
print("Finished incremental:", success)        # True if clean

# 5) Shutdown
client.shutdown_node()  # Graceful shutdown
```

### 4. Asserting New Knowledge

```python
from knowrob_ros.knowrob_ros_lib import (
    KnowRobRosLib,
    get_default_modalframe,
    TripleQueryBuilder,
    graph_answers_to_list
)
from knowrob_ros.msg import TellResult

# 1) Initialize client and modal frame
client = KnowRobRosLib()                          # Instantiate client
client.init_node("tutorial_client")             # Initialize ROS node
modal = get_default_modalframe()                  # Default modal frame

# 2) Build RDF-like triples to assert
builder = TripleQueryBuilder()                    # Helper to create Triple messages
builder.add("alice", "marriedTo", "frank")  # Define a new fact
triples = builder.get_triples()                   # Retrieve list of Triple messages

# 3) Send triples to KnowRob (Tell action)
tell_res = client.tell(triples, modal)            # Perform assertion
if tell_res.status == TellResult.TRUE:           # Check for success
    print("Successfully asserted marriage relationship.")
else:
    print("Assertion failed.")                   # Handle failure case

# 4) Verify assertion via AskAll query
check = client.ask_all(
    "marriedTo(alice, X)",  # Query newly asserted relationship
    modal                     # ModalFrame
)
bindings = graph_answers_to_list(check.answers)   # Convert results
print("Bindings:", bindings)                    # Expect X = frank

# 5) Shutdown
client.shutdown_node()  # Graceful shutdown
```
