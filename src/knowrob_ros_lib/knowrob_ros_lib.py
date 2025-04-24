import rospy
import actionlib
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
    ModalFrame
)


class KnowRobRosLib:
    def __init__(self):
        self._ask_one_client = None

    def init_node(self, name):
        rospy.init_node(name, anonymous=True)
        self._ask_one_client = actionlib.SimpleActionClient("knowrob/askone", AskOneAction)
        self._ask_one_client.wait_for_server()
        self._ask_all_client = actionlib.SimpleActionClient("knowrob/askall", AskAllAction)
        self._ask_all_client.wait_for_server()

    def shutdown_node(self):
        rospy.signal_shutdown("KnowRob node shutdown")
        if self._ask_one_client:
            self._ask_one_client.cancel_all_goals()

    def ask_one(self, query, modal_frame, lang=GraphQueryMessage.LANG_FOL):
        goal = AskOneGoal()
        goal.query.queryString = query
        goal.query.frame = modal_frame
        goal.query.lang = lang
        self._ask_one_client.send_goal(goal)
        self._ask_one_client.wait_for_result()
        result = self._ask_one_client.get_result()
        return result
    
    def ask_all(self, query, modal_frame, lang=GraphQueryMessage.LANG_FOL):
        goal = AskAllGoal()
        goal.query.queryString = query
        goal.query.frame = modal_frame
        goal.query.lang = lang
        self._ask_all_client.send_goal(goal)
        self._ask_all_client.wait_for_result()
        result = self._ask_all_client.get_result()
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

def get_default_modalframe():
    modalframe = ModalFrame()
    modalframe.epistemicOperator = ModalFrame.KNOWLEDGE
    modalframe.temporalOperator = ModalFrame.CURRENTLY
    modalframe.minPastTimestamp = ModalFrame.UNSPECIFIED_TIMESTAMP
    modalframe.maxPastTimestamp = ModalFrame.UNSPECIFIED_TIMESTAMP
    modalframe.confidence = 0.0
    return modalframe

def graph_answer_to_dict(answer_msg):
    """
    Convert a GraphAnswerMessage to a dictionary format. 
    The dictionary will have the keys as the variable names and the values as the corresponding values.
    """

    results = {}

    for binding_group in answer_msg.substitution:
        if binding_group.type == KeyValuePair.TYPE_STRING:
            results[binding_group.key] = binding_group.value_string
        elif binding_group.type == KeyValuePair.TYPE_FLOAT:
            results[binding_group.key] = binding_group.value_float
        elif binding_group.type == KeyValuePair.TYPE_INT:
            results[binding_group.key] = binding_group.value_int
        elif binding_group.type == KeyValuePair.TYPE_LONG:
            results[binding_group.key] = binding_group.value_long
        elif binding_group.type == KeyValuePair.TYPE_VARIABLE:
            results[binding_group.key] = binding_group.value_variable
        elif binding_group.type == KeyValuePair.TYPE_PREDICATE:
            results[binding_group.key] = binding_group.value_predicate
        elif binding_group.type == KeyValuePair.TYPE_LIST:
            # Lists are stored as a raw string and require custom parsing
            results[binding_group.key] = binding_group.value_list
        else:
            # Throw an error or handle unknown types
            raise ValueError(f"Unknown type: {binding_group.type}")

    return results

def graph_answers_to_list(answer_msgs):
    """
    Convert a list of GraphAnswerMessage to a list of dictionaries.
    Each dictionary will have the keys as the variable names and the values as the corresponding values.
    """
    results = []
    for answer_msg in answer_msgs:
        result = graph_answer_to_dict(answer_msg)
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
