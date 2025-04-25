#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
test_knowrob_ros_lib.py

Unit tests for KnowRobRosLib, now including incremental-query support.
"""

import unittest
import rosunit

from knowrob_ros.knowrob_ros_lib import (
    KnowRobRosLib,
    TripleQueryBuilder,
    graph_answer_to_dict,
    graph_answers_to_list,
    get_default_modalframe,
)
from knowrob_ros.msg import (
    AskAllResult,
    AskIncrementalResult,
    AskIncrementalNextSolutionResult,
    AskOneResult,
    TellResult,
)


class TestKnowrobRosLib(unittest.TestCase):
    """
    TestCase for KnowRobRosLib, covering AskOne, AskAll, AskIncremental, and Tell.
    """

    @classmethod
    def setUpClass(cls):
        """
        Initialize KnowRobRosLib and ROS node once for all tests.
        """
        cls.knowrob_ros = KnowRobRosLib()
        cls.knowrob_ros.init_node("test_knowrob_ros_lib")

    @classmethod
    def tearDownClass(cls):
        """
        Shutdown ROS node after all tests.
        """
        cls.knowrob_ros.shutdown_node()

    def test_ask_all(self):
        """AskAll should return all matches for a query."""
        result = self.knowrob_ros.ask_all(
            "lpn:jealous(lpn:vincent, X)",
            get_default_modalframe()
        )
        self.assertEqual(result.status, AskAllResult.TRUE)
        bindings = graph_answers_to_list(result.answers)
        self.assertEqual(bindings, [{
            'X': 'http://knowrob.org/kb/lpn#marsellus'
        }])

    def test_ask_one(self):
        """AskOne should return a single binding for a query."""
        result = self.knowrob_ros.ask_one(
            "lpn:jealous(lpn:vincent, X)",
            get_default_modalframe()
        )
        self.assertEqual(result.status, AskOneResult.TRUE)
        binding = graph_answer_to_dict(result.answer)
        self.assertEqual(binding, {
            'X': 'http://knowrob.org/kb/lpn#marsellus'
        })

    def test_tell(self):
        """Tell should insert triples and they should be queryable."""
        builder = TripleQueryBuilder()
        builder.add("alice", "marriedTo", "frank")
        triples = builder.get_triples()

        tell_result = self.knowrob_ros.tell(triples, get_default_modalframe())
        self.assertEqual(tell_result.status, TellResult.TRUE)

        query_result = self.knowrob_ros.ask_all(
            "marriedTo(alice, X)",
            get_default_modalframe()
        )
        bindings = graph_answers_to_list(query_result.answers)
        self.assertEqual(bindings, [{
            'X': 'frank'
        }])

    def test_ask_incremental(self):
        """
        Full incremental-query flow: start, get first solution, then finish.
        """
        # Start incremental query
        start = self.knowrob_ros.ask_incremental(
            "lpn:jealous(lpn:vincent, X)",
            get_default_modalframe()
        )
        self.assertEqual(start.status, AskIncrementalResult.TRUE)
        query_id = start.queryId
        self.assertGreater(query_id, 0)

        # Retrieve next (first) solution
        next_sol = self.knowrob_ros.next_solution(query_id)
        self.assertEqual(next_sol.status, AskIncrementalNextSolutionResult.TRUE)
        binding = graph_answer_to_dict(next_sol.answer)
        self.assertEqual(binding, {
            'X': 'http://knowrob.org/kb/lpn#marsellus'
        })

        # Finish incremental query
        finished = self.knowrob_ros.finish_incremental(query_id)
        self.assertTrue(finished)


# Note: stray free-standing setUpClass below is a duplicate and has no effect on tests.
@classmethod
def setUpClass(cls):
    cls.knowrob_ros = KnowRobRosLib()
    cls.knowrob_ros.init_node("test_knowrob_ros_lib")


if __name__ == '__main__':
    rosunit.unitrun(
        'knowrob_ros',           # package name
        'test_knowrob_ros_lib',  # test name
        TestKnowrobRosLib        # TestCase class
    )
