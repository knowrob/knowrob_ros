from knowrob_ros.knowrob_ros_lib import KnowRobRosLib, graph_answer_to_dict, get_default_modalframe, graph_answers_to_list
import unittest
import rosunit
from knowrob_ros.msg import (
    KeyValuePair,
    AskOneAction,
    AskOneGoal,
    AskOneResult,
    AskAllAction,
    AskAllGoal,
    AskAllResult,
    GraphQueryMessage,
    GraphAnswerMessage,
)

class TestKnowrobRosLib(unittest.TestCase):
    def test_ask_all(self):
        # Test the ask_one function
        ask_all_result = self.knowrob_ros.ask_all("lpn:jealous(lpn:vincent, X)", get_default_modalframe())
        self.assertTrue(ask_all_result.status == AskAllResult.TRUE)
        result_dict = graph_answers_to_list(ask_all_result.answers)
        print("Result dict:", str(result_dict))
        self.assertEqual(result_dict, [{
            'X': 'http://knowrob.org/kb/lpn#marsellus'
        }])

    def test_ask_one(self):
        # Test the ask_one function
        ask_one_result = self.knowrob_ros.ask_one("lpn:jealous(lpn:vincent, X)", get_default_modalframe())
        self.assertTrue(ask_one_result.status == AskOneResult.TRUE)
        result_dict = graph_answer_to_dict(ask_one_result.answer)
        print("Result dict:", str(result_dict))
        self.assertEqual(result_dict, {
            'X': 'http://knowrob.org/kb/lpn#marsellus'
        })

    # def test_tell(self):
    #     # Create the triples to be added
    #     builder = knowrob_ros_lib.TripleQueryBuilder()
    #     builder.add("alice", "knows", "bob")
    #     builder.add("bob", "likes", "pizza")
    #     query_str = builder.build_query_string()

    #     # Test the tell function
    #     result = knowrob_ros.tell(query_str)
    #     self.assertTrue(result.success)
    #     result = knowrob_ros.ask_all("lpn:jelous(alice, X)")
    #     self.assertEqual(result.bindings, [{
    #         'X': 'pizza'
    #     }])

    # Init the test class
    @classmethod
    def setUpClass(cls):
        # Initialize the knowrob_ros_lib
        cls.knowrob_ros = KnowRobRosLib()
        # Initialize the ROS node
        cls.knowrob_ros.init_node("test_knowrob_ros_lib")

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS node
        cls.knowrob_ros.shutdown_node()
        
@classmethod
def setUpClass(cls):
    cls.knowrob_ros = KnowRobRosLib()
    cls.knowrob_ros.init_node("test_knowrob_ros_lib")


if __name__ == '__main__':
    rosunit.unitrun(
        'knowrob_ros',           # your package
        'test_knowrob_ros_lib',  # test name
        TestKnowrobRosLib        # your TestCase
    )