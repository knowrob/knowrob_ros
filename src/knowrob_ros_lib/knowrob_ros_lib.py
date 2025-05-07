#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
knowrob_ros_lib.py

A simple wrapper around KnowRob ROS actionlib services.
Provides methods to call AskOne, AskAll, AskIncremental, and Tell actions,
as well as utility functions for working with modal frames and GraphAnswerMessages.
Now decoupled from rospy.init_node() to allow safer integration in ROS nodes.
"""

import actionlib
import rospy

from knowrob_ros.msg import (
    AskAllAction, AskAllGoal, AskAllResult,
    AskIncrementalAction, AskIncrementalGoal,
    AskIncrementalNextSolutionAction, AskIncrementalNextSolutionGoal,
    AskOneAction, AskOneGoal, AskOneResult,
    GraphAnswerMessage, GraphQueryMessage, KeyValuePair, ModalFrame,
    TellAction, TellGoal, TellResult, Triple,
)
from knowrob_ros.srv import AskIncrementalFinish


class KnowRobRosLib:
    """
    Wrapper for KnowRob ROS services using actionlib.

    Usage:
        roslib = KnowRobRosLib()
        roslib.init_clients()  # After rospy.init_node()
        result = roslib.ask_one("someQuery", get_default_modalframe())
    """

    def __init__(self):
        """
        Initialize client and service placeholders.
        Use init_clients() to connect to action servers.
        """
        self._ask_one_client = None
        self._ask_all_client = None
        self._ask_incremental_client = None
        self._ask_incremental_next_client = None
        self._tell_client = None
        self._ask_incremental_finish = None

    def init_clients(self):
        """
        Initialize all actionlib clients and service proxies.
        Does NOT call rospy.init_node(). Call that yourself before this.
        """
        rospy.loginfo("Initializing KnowRob ROS action clients...")

        self._ask_one_client = actionlib.SimpleActionClient('knowrob/askone', AskOneAction)
        self._ask_all_client = actionlib.SimpleActionClient('knowrob/askall', AskAllAction)
        self._ask_incremental_client = actionlib.SimpleActionClient('knowrob/askincremental', AskIncrementalAction)
        self._ask_incremental_next_client = actionlib.SimpleActionClient(
            'knowrob/askincremental_next_solution', AskIncrementalNextSolutionAction)
        self._tell_client = actionlib.SimpleActionClient('knowrob/tell', TellAction)

        # Wait for all servers
        self._ask_one_client.wait_for_server()
        self._ask_all_client.wait_for_server()
        self._ask_incremental_client.wait_for_server()
        self._ask_incremental_next_client.wait_for_server()
        self._tell_client.wait_for_server()

        rospy.loginfo("Waiting for KnowRob service: askincremental_finish...")
        rospy.wait_for_service('knowrob/askincremental_finish')
        self._ask_incremental_finish = rospy.ServiceProxy('knowrob/askincremental_finish', AskIncrementalFinish)

        rospy.loginfo("KnowRob action clients and services initialized.")

    def shutdown(self):
        """
        Cancel pending goals. Optionally call rospy.signal_shutdown separately if needed.
        """
        for client in (
            self._ask_one_client,
            self._ask_all_client,
            self._ask_incremental_client,
            self._ask_incremental_next_client,
            self._tell_client,
        ):
            if client:
                client.cancel_all_goals()

    def ask_one(self, query, modal_frame, lang=GraphQueryMessage.LANG_FOL):
        """
        Send an AskOne query to KnowRob and wait for a single result.

        Args:
            query (str): Query string in FOL or Prolog syntax.
            modal_frame (ModalFrame): Context of the query.
            lang (int): Query language (default: LANG_FOL).

        Returns:
            AskOneResult
        """
        goal = AskOneGoal()
        goal.query.queryString = query
        goal.query.frame = modal_frame
        goal.query.lang = lang

        self._ask_one_client.send_goal(goal)
        self._ask_one_client.wait_for_result()
        return self._ask_one_client.get_result()

    def ask_all(self, query, modal_frame, lang=GraphQueryMessage.LANG_FOL):
        """
        Send an AskAll query to KnowRob and wait for all matching results.

        Returns:
            AskAllResult
        """
        goal = AskAllGoal()
        goal.query.queryString = query
        goal.query.frame = modal_frame
        goal.query.lang = lang

        self._ask_all_client.send_goal(goal)
        self._ask_all_client.wait_for_result()
        return self._ask_all_client.get_result()

    def ask_incremental(self, query, modal_frame, lang=GraphQueryMessage.LANG_FOL):
        """
        Start an incremental query. The server returns a queryId
        to retrieve solutions one-by-one.

        Returns:
            AskIncrementalResult
        """
        goal = AskIncrementalGoal()
        goal.query.queryString = query
        goal.query.frame = modal_frame
        goal.query.lang = lang

        self._ask_incremental_client.send_goal(goal)
        self._ask_incremental_client.wait_for_result()
        return self._ask_incremental_client.get_result()

    def next_solution(self, query_id):
        """
        Retrieve the next solution from an incremental query.

        Args:
            query_id (int): ID returned by ask_incremental()

        Returns:
            AskIncrementalNextSolutionResult
        """
        goal = AskIncrementalNextSolutionGoal()
        goal.queryId = query_id

        self._ask_incremental_next_client.send_goal(goal)
        self._ask_incremental_next_client.wait_for_result()
        return self._ask_incremental_next_client.get_result()

    def finish_incremental(self, query_id):
        """
        Finish an incremental query and release server-side resources.

        Returns:
            bool: Success status.
        """
        return self._ask_incremental_finish(query_id).success

    def tell(self, list_of_triples, modal_frame):
        """
        Send RDF-style triples to KnowRob to assert knowledge.

        Args:
            list_of_triples (list of Triple): Triples to insert.
            modal_frame (ModalFrame): Context.

        Returns:
            TellResult
        """
        goal = TellGoal()
        goal.tell.triples = list_of_triples
        goal.tell.frame = modal_frame

        self._tell_client.send_goal(goal)
        self._tell_client.wait_for_result()
        return self._tell_client.get_result()


# ------------------------------
# Helper Functions
# ------------------------------

def get_default_modalframe():
    """
    Create a default ModalFrame with epistemic and temporal context.

    Returns:
        ModalFrame
    """
    modal_frame = ModalFrame()
    modal_frame.epistemicOperator = ModalFrame.KNOWLEDGE
    modal_frame.temporalOperator = ModalFrame.CURRENTLY
    modal_frame.minPastTimestamp = ModalFrame.UNSPECIFIED_TIMESTAMP
    modal_frame.maxPastTimestamp = ModalFrame.UNSPECIFIED_TIMESTAMP
    modal_frame.confidence = 0.0
    return modal_frame


def graph_answer_to_dict(answer_msg):
    """
    Convert a GraphAnswerMessage into a Python dictionary.

    Args:
        answer_msg (GraphAnswerMessage)

    Returns:
        dict: Variable bindings.
    """
    results = {}
    for kv in answer_msg.substitution:
        if kv.type == KeyValuePair.TYPE_STRING:
            results[kv.key] = kv.value_string
        elif kv.type == KeyValuePair.TYPE_FLOAT:
            results[kv.key] = kv.value_float
        elif kv.type == KeyValuePair.TYPE_INT:
            results[kv.key] = kv.value_int
        elif kv.type == KeyValuePair.TYPE_LONG:
            results[kv.key] = kv.value_long
        elif kv.type == KeyValuePair.TYPE_VARIABLE:
            results[kv.key] = kv.value_variable
        elif kv.type == KeyValuePair.TYPE_PREDICATE:
            results[kv.key] = kv.value_predicate
        elif kv.type == KeyValuePair.TYPE_LIST:
            results[kv.key] = kv.value_list
        else:
            raise ValueError(f"Unknown KeyValuePair type: {kv.type}")
    return results


def graph_answers_to_list(answer_msgs):
    """
    Convert a list of GraphAnswerMessages to a list of dicts.

    Returns:
        list of dict
    """
    return [graph_answer_to_dict(msg) for msg in answer_msgs]


class TripleQueryBuilder:
    """
    Helper class to construct lists of Triple messages for Tell.

    Example:
        builder = TripleQueryBuilder()
        builder.add("s", "p", "o")
        triples = builder.get_triples()
    """

    def __init__(self):
        self.triples = []

    def add(self, subject, predicate, obj):
        triple = Triple()
        triple.subject = subject
        triple.predicate = predicate
        triple.object = obj
        self.triples.append(triple)

    def get_triples(self):
        return self.triples
