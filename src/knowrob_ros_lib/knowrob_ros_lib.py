import rospy
import actionlib
from knowrob_askone.msg import (
    KeyValuePair,
    AskOneAction,
    AskOneGoal,
    GraphQueryMessage,
    GraphAnswerMessage,
)


class KnowRobRosLib:
    def __init__(self):
        self._ask_one_client = None

    def init_node(self, name):
        rospy.init_node(name, anonymous=True)
        self._ask_one_client = actionlib.SimpleActionClient("knowrob/ask_one", AskOneAction)
        self._ask_one_client.wait_for_server()

    def shutdown_node(self):
        rospy.signal_shutdown("KnowRob node shutdown")
        if self._ask_one_client:
            self._ask_one_client.cancel_all_goals()

    def ask_one(self, query):
        goal = AskOneGoal()
        goal.query.query_string = query
        self._ask_one_client.send_goal(goal)
        self._ask_one_client.wait_for_result()
        result = self._ask_one_client.get_result()
        return result

    # def tell(self, triples_str):
    #     request = TellRequest()
    #     request.query.query_string = triples_str
    #     response = self._tell_service(request)
    #     return TellResultAdapter(response)

    # def ask_all(self, query):
    #     # This is a stub assuming synchronous call, you'd use ROS service or action here too
    #     # Replace with actual implementation for asking all
    #     return GraphResultAdapter(GraphAnswerMessage(bindings=[KeyValuePair(key="X", value="hans")]))

    def graph_answer_to_dicts(answer_msg):
        """
        Convert a GraphAnswerMessage to a list of dictionaries.
        Each dictionary represents one solution, where keys are variable names,
        and values are decoded based on the type field.
        """

        results = []

        for binding_group in answer_msg.bindings:
            result = {}
            for pair in binding_group.bindings:
                if pair.type == KeyValuePair.TYPE_STRING:
                    result[pair.key] = pair.value_string
                elif pair.type == KeyValuePair.TYPE_FLOAT:
                    result[pair.key] = pair.value_float
                elif pair.type == KeyValuePair.TYPE_INT:
                    result[pair.key] = pair.value_int
                elif pair.type == KeyValuePair.TYPE_LONG:
                    result[pair.key] = pair.value_long
                elif pair.type == KeyValuePair.TYPE_VARIABLE:
                    result[pair.key] = pair.value_variable
                elif pair.type == KeyValuePair.TYPE_PREDICATE:
                    result[pair.key] = pair.value_predicate
                elif pair.type == KeyValuePair.TYPE_LIST:
                    # Lists are stored as a raw string and require custom parsing
                    result[pair.key] = pair.value_list
                else:
                    result[pair.key] = None  # Unknown type
            results.append(result)

        return results

# class GraphResultAdapter:
#     def __init__(self, msg):
#         self.bindings = (
#             {kv.key: kv.value for kv in msg.bindings}
#             if isinstance(msg.bindings, list)
#             else msg.bindings
#         )


# class TellResultAdapter:
#     def __init__(self, response):
#         self.success = response.success


class TripleQueryBuilder:
    def __init__(self):
        self.triples = []

    def add(self, subject, predicate, obj):
        """Add a triple to the list."""
        self.triples.append((subject, predicate, obj))

    def build_query_string(self):
        """Generate a Prolog-style query string."""
        return ', '.join(f'{pred}({subj},{obj})' for subj, pred, obj in self.triples)


# Module-level functions
_knowrob_instance = KnowRobRosLib()


def init_node(name):
    _knowrob_instance.init_node(name)


def shutdown_node():
    _knowrob_instance.shutdown_node()


def ask_one(query):
    return _knowrob_instance.ask_one(query)


# def ask_all(query):
#     return _knowrob_instance.ask_all(query)


# def tell(triples_str):
#     return _knowrob_instance.tell(triples_str)
