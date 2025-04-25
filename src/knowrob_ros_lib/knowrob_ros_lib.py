#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
knowrob_ros_lib.py

A simple wrapper around KnowRob ROS actionlib services.
Provides methods to initialize the ROS node and to call AskOne, AskAll, AskIncremental, and Tell actions,
as well as utility functions for working with modal frames and GraphAnswerMessages.
"""

import actionlib
import rospy

from knowrob_ros.msg import (
    AskAllAction,
    AskAllGoal,
    AskAllResult,
    AskIncrementalAction,
    AskIncrementalGoal,
    AskIncrementalNextSolutionAction,
    AskIncrementalNextSolutionGoal,
    AskOneAction,
    AskOneGoal,
    AskOneResult,
    GraphAnswerMessage,
    GraphQueryMessage,
    KeyValuePair,
    ModalFrame,
    TellAction,
    TellGoal,
    TellResult,
    Triple,
)
from knowrob_ros.srv import AskIncrementalFinish


class KnowRobRosLib:
    """
    Wrapper for KnowRob ROS services using actionlib.

    Methods:
        init_node(name): Initialize ROS node, action clients, and services.
        shutdown_node(): Shutdown ROS node and cancel all goals.
        ask_one(query, modal_frame, lang): Single-result query.
        ask_all(query, modal_frame, lang): Multi-result query.
        ask_incremental(query, modal_frame, lang): Start an incremental query.
        next_solution(query_id): Retrieve next solution from an incremental query.
        finish_incremental(query_id): Finish an incremental query.
        tell(triples, modal_frame): Assert triples into the knowledge base.
    """

    def __init__(self):
        """
        Initialize client and service placeholders. Call init_node() before use.
        """
        self._ask_one_client = None
        self._ask_all_client = None
        self._ask_incremental_client = None
        self._ask_incremental_next_client = None
        self._tell_client = None
        self._ask_incremental_finish = None

    def init_node(self, name):
        """
        Initialize the ROS node and all actionlib clients and services.

        Args:
            name (str): Name for the ROS node.
        """
        rospy.init_node(name, anonymous=True)

        # AskOne
        self._ask_one_client = actionlib.SimpleActionClient(
            'knowrob/askone', AskOneAction)
        self._ask_one_client.wait_for_server()

        # AskAll
        self._ask_all_client = actionlib.SimpleActionClient(
            'knowrob/askall', AskAllAction)
        self._ask_all_client.wait_for_server()

        # AskIncremental (start)
        self._ask_incremental_client = actionlib.SimpleActionClient(
            'knowrob/askincremental', AskIncrementalAction)
        self._ask_incremental_client.wait_for_server()

        # AskIncremental (next solution)
        self._ask_incremental_next_client = actionlib.SimpleActionClient(
            'knowrob/askincremental_next_solution', AskIncrementalNextSolutionAction)
        self._ask_incremental_next_client.wait_for_server()

        # Tell
        self._tell_client = actionlib.SimpleActionClient(
            'knowrob/tell', TellAction)
        self._tell_client.wait_for_server()

        # Finish incremental query service
        rospy.wait_for_service('knowrob/askincremental_finish')
        self._ask_incremental_finish = rospy.ServiceProxy(
            'knowrob/askincremental_finish', AskIncrementalFinish)

    def shutdown_node(self):
        """
        Shutdown the ROS node and cancel any pending goals.
        """
        rospy.signal_shutdown('KnowRob node shutdown')

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
        Start an incremental query. The server returns a status and a queryId
        that can be used to fetch solutions one by one.

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
        Retrieve the next solution for an active incremental query.

        Args:
            query_id (int): ID from ask_incremental().

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
        Finish an incremental query, releasing server-side resources.

        Args:
            query_id (int): ID from ask_incremental().

        Returns:
            bool: True if the finish call succeeded.
        """
        resp = self._ask_incremental_finish(query_id)
        return resp.success

    def tell(self, list_of_triples, modal_frame):
        """
        Send a set of RDF-style triples to the KnowRob knowledge base.

        Returns:
            TellResult
        """
        goal = TellGoal()
        goal.tell.triples = list_of_triples
        goal.tell.frame = modal_frame

        self._tell_client.send_goal(goal)
        self._tell_client.wait_for_result()
        return self._tell_client.get_result()


def get_default_modalframe():
    """
    Create a default ModalFrame with knowledge, current time, and unspecified timestamps.

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
    Convert a GraphAnswerMessage into a Python dict mapping variable names to values.
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
    Convert multiple GraphAnswerMessages into a list of dicts.
    """
    return [graph_answer_to_dict(msg) for msg in answer_msgs]


class TripleQueryBuilder:
    """
    Helper to build lists of Triple messages for assertions.
    Usage:
        builder = TripleQueryBuilder()
        builder.add(subject, predicate, object)
        triples = builder.get_triples()
    """

    def __init__(self):
        self.triples = []

    def add(self, subject, predicate, obj):
        """
        Add a new Triple to the builder.

        Args:
            subject (str)
            predicate (str)
            obj (str)
        """
        triple = Triple()
        triple.subject = subject
        triple.predicate = predicate
        triple.object = obj
        self.triples.append(triple)

    def get_triples(self):
        """
        Return the collected triples.
        """
        return self.triples
