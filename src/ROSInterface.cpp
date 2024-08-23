/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#include "ROSInterface.h"
// KnowRob
#include "knowrob/knowrob.h"
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/queries/QueryError.h"
#include "knowrob/formulas/ModalFormula.h"
#include "knowrob/terms/ListTerm.h"
#include "knowrob/queries/QueryTree.h"
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <knowrob_ros/GraphAnswerMessage.h>
#include <knowrob_ros/GraphQueryMessage.h>
#include <knowrob_ros/KeyValuePair.h>
#include <knowrob_ros/AskAllAction.h>
#include "knowrob/integration/InterfaceUtils.h"
#include <boost/property_tree/json_parser.hpp>
#include <utility>

using namespace knowrob;
using namespace knowrob_ros;

ROSInterface::ROSInterface(const boost::property_tree::ptree &config)
		: askall_action_server_(nh_, "knowrob/askall", boost::bind(&ROSInterface::executeAskAllCB, this, _1), false),
		  askone_action_server_(nh_, "knowrob/askone", boost::bind(&ROSInterface::executeAskOneCB, this, _1), false),
		  askincremental_action_server_(nh_, "knowrob/askincremental",
										boost::bind(&ROSInterface::executeAskIncrementalCB, this, _1), false),
		  askincremental_next_solution_action_server_(nh_, "knowrob/askincremental_next_solution",
													  boost::bind(&ROSInterface::executeAskIncrementalNextSolutionCB,
																  this, _1), false),
		  tell_action_server_(nh_, "knowrob/tell", boost::bind(&ROSInterface::executeTellCB, this, _1), false),
		  kb_(KnowledgeBase::create(config)) {
	// Start all action servers
	askall_action_server_.start();
	askone_action_server_.start();
	askincremental_action_server_.start();
	askincremental_next_solution_action_server_.start();
	tell_action_server_.start();
	ask_incremental_finish_service_ = nh_.advertiseService("knowrob/askincremental_finish",
														   &ROSInterface::handleAskIncrementalFinish, this);
}

ROSInterface::~ROSInterface() = default;

// Function to convert GraphQueryMessage to std::unordered_map
std::unordered_map<std::string, boost::any> ROSInterface::translateGraphQueryMessage(const GraphQueryMessage &query) {
	std::unordered_map<std::string, boost::any> options;

	options["epistemicOperator"] = int(query.epistemicOperator);
	options["aboutAgentIRI"] = query.aboutAgentIRI;
	options["confidence"] = query.confidence;
	options["temporalOperator"] = int(query.temporalOperator);
	options["minPastTimestamp"] = query.minPastTimestamp;
	options["maxPastTimestamp"] = query.maxPastTimestamp;

	return options;
}

GraphAnswerMessage ROSInterface::createGraphAnswer(std::shared_ptr<const AnswerYes> answer) {
	const BindingsPtr &substitution = answer->substitution();
	GraphAnswerMessage graphAnswer;
	for (const auto &pair: *substitution) {
		KeyValuePair kvpair;
		kvpair.key = pair.first;
		TermPtr term = pair.second.second;
		// Stringstream for list terms
		std::stringstream ss;

		if (term->termType() == TermType::ATOMIC) {
			auto atomic = std::static_pointer_cast<Atomic>(term);
			switch (atomic->atomicType()) {
				case AtomicType::STRING:
				case AtomicType::ATOM:
					kvpair.type = KeyValuePair::TYPE_STRING;
					kvpair.value_string = atomic->stringForm().data();
					break;
				case AtomicType::NUMERIC: {
					auto numeric = std::static_pointer_cast<Numeric>(atomic);
					switch (numeric->xsdType()) {
						case XSDType::FLOAT:
						case XSDType::DOUBLE:
							kvpair.type = KeyValuePair::TYPE_FLOAT;
							kvpair.value_float = numeric->asDouble();
							break;
						case XSDType::NON_NEGATIVE_INTEGER:
						case XSDType::UNSIGNED_INT:
						case XSDType::INTEGER:
							kvpair.type = KeyValuePair::TYPE_INT;
							kvpair.value_int = numeric->asInteger();
							break;
						case XSDType::UNSIGNED_LONG:
						case XSDType::LONG:
							kvpair.type = KeyValuePair::TYPE_LONG;
							kvpair.value_long = numeric->asLong();
							break;
						case XSDType::UNSIGNED_SHORT:
						case XSDType::SHORT:
							kvpair.type = KeyValuePair::TYPE_INT;
							kvpair.value_int = numeric->asShort();
							break;
						case XSDType::BOOLEAN:
						case XSDType::STRING:
						case XSDType::LAST:
							break;
					}
					break;
				}
			}
		} else if (term->termType() == TermType::FUNCTION) {
			// TODO: Can this happen? If yes implement it
		} else if (term->termType() == TermType::VARIABLE) {
			// TODO: Can this happen? If yes implement it
		}

		graphAnswer.substitution.push_back(kvpair);
	}
	return graphAnswer;
}

void ROSInterface::executeAskAllCB(const AskAllGoalConstPtr &goal) {

	// Implement your action here
	FormulaPtr phi(QueryParser::parse(goal->query.queryString));

	FormulaPtr mPhi = InterfaceUtils::applyModality(translateGraphQueryMessage(goal->query), phi);

	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
	auto resultStream = kb_->submitQuery(mPhi, ctx);
	auto resultQueue = resultStream->createQueue();

	int numSolutions_ = 0;
	AskAllResult result;
	while (true) {
		auto nextResult = resultQueue->pop_front();

		if (nextResult->indicatesEndOfEvaluation()) {
			break;
		} else if (nextResult->tokenType() == TokenType::ANSWER_TOKEN) {
			auto answer = std::static_pointer_cast<const Answer>(nextResult);
			if (answer->isPositive()) {
				auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);
				if (positiveAnswer->substitution()->empty()) {
					numSolutions_ = 1;
					break;
				} else {
					// Push one answer
					GraphAnswerMessage graphAns = createGraphAnswer(positiveAnswer);
					result.answer.push_back(graphAns);
					numSolutions_ += 1;
					// publish feedback
					AskAllFeedback feedback;
					feedback.numberOfSolutions = numSolutions_;
					askall_action_server_.publishFeedback(feedback);
				}
			}
		}
	}

	if (numSolutions_ == 0) {
		result.status = AskAllResult::FALSE;
	} else {
		result.status = AskAllResult::TRUE;
	}
	askall_action_server_.setSucceeded(result);
}

void ROSInterface::executeAskIncrementalCB(const AskIncrementalGoalConstPtr &goal) {
	std::lock_guard<std::mutex> lock(query_mutex_);

	// Implement your action here
	FormulaPtr phi(QueryParser::parse(goal->query.queryString));

	FormulaPtr mPhi = InterfaceUtils::applyModality(translateGraphQueryMessage(goal->query), phi);

	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
	auto resultStream = kb_->submitQuery(mPhi, ctx);
	auto resultQueue = resultStream->createQueue();

	// Store result queue for execution of next solution
	query_results_[next_query_id_] = resultQueue;

	// Result, feedback
	AskIncrementalResult result;
	AskIncrementalFeedback feedback;

	// Publish feedback
	feedback.finished = true;
	askincremental_action_server_.publishFeedback(feedback);

	// Publish result
	result.queryId = next_query_id_;
	result.status = AskIncrementalResult::TRUE;
	askincremental_action_server_.setSucceeded(result);

	next_query_id_ += 1;
}

void ROSInterface::executeAskIncrementalNextSolutionCB(const AskIncrementalNextSolutionGoalConstPtr &goal) {
	// Lock mutex
	std::lock_guard<std::mutex> lock(query_mutex_);

	// Define feedback, result and isTrue
	AskIncrementalNextSolutionFeedback feedback;
	AskIncrementalNextSolutionResult result;
	bool isTrue = false;

	// Get result queue for query ID
	auto resultQueue = query_results_[goal->queryId];

	// Check if query ID is valid
	if (resultQueue == nullptr) {
		result.status = AskIncrementalNextSolutionResult::INVALID_QUERY_ID;
		askincremental_next_solution_action_server_.setAborted(result);
		return;
	}

	// Retrieve next solution
	auto nextResult = resultQueue->pop_front();
	if (nextResult->tokenType() == TokenType::ANSWER_TOKEN) {
		auto answer = std::static_pointer_cast<const Answer>(nextResult);
		if (answer->isPositive()) {
			auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);
			isTrue = true;
			if (!positiveAnswer->substitution()->empty()) {
				// Publish feedback
				result.answer = createGraphAnswer(positiveAnswer);
			}
		}
	} else if (nextResult->indicatesEndOfEvaluation()) {
		// Remove id
		query_results_.erase(goal->queryId);
	}

	// If there is no next solution set status to false
	if (isTrue) {
		result.status = AskIncrementalNextSolutionResult::TRUE;
	} else {
		result.status = AskIncrementalNextSolutionResult::FALSE;
	}

	// Publish finish feedback
	feedback.finished = true;
	askincremental_next_solution_action_server_.publishFeedback(feedback);
	// Publish result
	askincremental_next_solution_action_server_.setSucceeded(result);
	
}

bool ROSInterface::handleAskIncrementalFinish(AskIncrementalFinish::Request &req,
											  AskIncrementalFinish::Response &res) {
	std::lock_guard<std::mutex> lock(query_mutex_);

	// Check if the query ID exists
	auto it = query_results_.find(req.queryId);
	if (it != query_results_.end()) {
		// Remove the query ID from the map
		query_results_.erase(it);
		res.success = true;
	} else {
		res.success = false;
	}

	return true;
}

void ROSInterface::executeAskOneCB(const AskOneGoalConstPtr &goal) {
	FormulaPtr phi(QueryParser::parse(goal->query.queryString));

	FormulaPtr mPhi = InterfaceUtils::applyModality(translateGraphQueryMessage(goal->query), phi);

	auto ctx = std::make_shared<QueryContext>(QUERY_FLAG_ALL_SOLUTIONS);
	auto resultStream = kb_->submitQuery(mPhi, ctx);
	auto resultQueue = resultStream->createQueue();

	AskOneResult result;
	auto nextResult = resultQueue->pop_front();

	if (nextResult->indicatesEndOfEvaluation()) {
		result.status = AskOneResult::FALSE;
	} else if (nextResult->tokenType() == TokenType::ANSWER_TOKEN) {
		auto answer = std::static_pointer_cast<const Answer>(nextResult);
		if (answer->isPositive()) {
			auto positiveAnswer = std::static_pointer_cast<const AnswerYes>(answer);
			result.status = AskOneResult::TRUE;
			GraphAnswerMessage answer = createGraphAnswer(positiveAnswer);
			result.answer = answer;
		}
	}

	// Publish feedback
	AskOneFeedback feedback;
	feedback.finished = true;
	askone_action_server_.publishFeedback(feedback);
	// Publish result
	askone_action_server_.setSucceeded(result);
}

void ROSInterface::executeTellCB(const TellGoalConstPtr &goal) {
	FormulaPtr phi(QueryParser::parse(goal->query.queryString));

	FormulaPtr mPhi = InterfaceUtils::applyModality(translateGraphQueryMessage(goal->query), phi);

	bool success = InterfaceUtils::assertStatements(kb_, {mPhi});

	TellResult result;
	TellFeedback feedback;
	if (success) {
		result.status = TellResult::TRUE;
	} else {
		result.status = TellResult::TELL_FAILED;
	}
	feedback.finished = true;
	tell_action_server_.publishFeedback(feedback);
	tell_action_server_.setSucceeded(result);
}

int main(int argc, char **argv) {
	InitKnowRob(argc, argv);

	// Load settings files
	try {
		ros::init(argc, argv, "knowrob_node");
		ROSInterface ros_interface(InterfaceUtils::loadSettings());
		KB_INFO("[KnowRob] ROS node started.");
		ros::spin();
	}
	catch (std::exception &e) {
		KB_ERROR("an exception occurred: {}.", e.what());
		return EXIT_FAILURE;
	}
}
