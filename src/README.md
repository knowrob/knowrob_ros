\page ROS Interface Documentation

\section overview_sec Overview

The ROS Interface class provides a connection between ROS (Robot Operating System)
environments and the KnowRob knowledge base.
\section prereq_sec Prerequisites

Before using the ROSInterface, ensure that the following requirements are met:
- ROS Noetic is installed. (This version is tested and recommended for compatibility.

\section actions_sec Actions Provided

`ROSInterface` provides several actions to interact with the KnowRob system. These actions are as follows:

\subsection askall_subsec 1. AskAll Action

- <b>Action Name:</b> `/knowrob/askall`
- <b>Purpose:</b> Executes a query and retrieves all possible solutions.
- <b>Input:</b> `GraphQueryMessage` which includes the query string and parameters.
- <b>Output:</b> Returns all possible answers to the query.

\subsection askone_subsec 2. AskOne Action

- <b>Action Name:</b> `/knowrob/askone`
- <b>Purpose:</b> Executes a query and retrieves the first valid solution.
- <b>Input:</b> `GraphQueryMessage` as the query.
- <b>Output:</b> Returns the first possible answer if available.

\subsection askincr_subsec 3. AskIncremental Action

- <b>Action Name:</b> `/knowrob/askincremental`
- <b>Purpose:</b> Provides a way to incrementally retrieve answers to a query.
- <b>Input:</b> `GraphQueryMessage` containing the query details.
- <b>Output:</b> Returns an queryId to identify, that can be used to fetch subsequent solutions using the `askincremental_next_solution` action.

\subsection askincrnext_subsec 4. AskIncrementalNextSolution Action

- <b>Action Name:</b> `/knowrob/askincremental_next_solution`
- <b>Purpose:</b> Fetches the next available solution for an ongoing incremental query.
- <b>Input:</b> `queryId` to identify the query session.
- <b>Output:</b> The next solution or a status indicating no more solutions are available.

\subsection tell_subsec 5. Tell Action

- <b>Action Name:</b> `/knowrob/tell`
- <b>Purpose:</b> Submits knowledge to be asserted into the knowledge base.
- <b>Input:</b> `GraphQueryMessage` which includes statements to be asserted.
- <b>Output:</b> Status indicating success or failure of the operation.

\section usage_sec Usage

To utilize the ROSInterface run:

```bash
roslaunch knowrob knowrob.launch
```