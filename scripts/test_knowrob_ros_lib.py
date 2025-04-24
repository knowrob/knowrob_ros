import knowrob_ros.knowrob_ros_lib
import unittest
from knowrob_ros import AskOneAction, AskOneGoal  # Replace with actual import paths



class TestKnowrobRosLib(unittest.TestCase):
    # def test_ask_all(self):
    #     # Test the ask_all function
    #     result = knowrob_ros.ask_all("lpn:jelous(vincent, X)")
    #     self.assertEqual(result.bindings, [{
    #         'X': 'hans'
    #     }])

    def test_ask_one(self):
        # Test the ask_one function
        ask_one_result = knowrob_ros.ask_one("lpn:jealous(lpn:vincent, X)")
        self.assertTrue(ask_one_result.status == AskOneGoal.TRUE)
        result = knowrob_ros_lib.graph_answer_to_dicts(ask_one_result.answer)
        self.assertEqual(result.bindings, [{
            'X': 'lpn:marsellus'
        }])

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
    def setUp(self):
        # Initialize the knowrob_ros_lib
        self.knowrob_ros = knowrob_ros_lib.KnowRobRosLib()
        # Initialize the ROS node
        self.knowrob_ros.init_node("test_knowrob_ros_lib")

    def tearDown(self):
        # Shutdown the ROS node
        self.knowrob_ros.shutdown_node()

if __name__ == '__main__':
    unittest.main()