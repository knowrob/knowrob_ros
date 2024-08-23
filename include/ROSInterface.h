/*
 * Copyright (c) 2023, Sascha Jongebloed
 * All rights reserved.
 *
 * This file is part of KnowRob, please consult
 * https://github.com/knowrob/knowrob for license details.
 */

#ifndef KNOWROB_ROSINTERFACE_H
#define KNOWROB_ROSINTERFACE_H

// KnowRob
#include "knowrob/knowrob.h"
#include "knowrob/Logger.h"
#include "knowrob/KnowledgeBase.h"
#include "knowrob/queries/QueryParser.h"
#include "knowrob/formulas/ModalFormula.h"
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <knowrob/GraphAnswerMessage.h>
#include <knowrob/GraphQueryMessage.h>
#include <knowrob/KeyValuePair.h>
#include <knowrob/AskAllAction.h>
#include <knowrob/AskOneAction.h>
#include <knowrob/AskIncrementalAction.h>
#include <knowrob/AskIncrementalNextSolutionAction.h>
#include <knowrob/AskIncrementalFinish.h>
#include <knowrob/TellAction.h>
#include <actionlib/server/simple_action_server.h>
// std
#include <mutex>

namespace knowrob {
	class ROSInterface {
	private:
		ros::NodeHandle nh_;

		// Action Servers
		actionlib::SimpleActionServer<AskAllAction> askall_action_server_;
		actionlib::SimpleActionServer<AskOneAction> askone_action_server_;
		actionlib::SimpleActionServer<AskIncrementalAction> askincremental_action_server_;
		actionlib::SimpleActionServer<AskIncrementalNextSolutionAction> askincremental_next_solution_action_server_;
		actionlib::SimpleActionServer<TellAction> tell_action_server_;

		// ROS Services
		ros::ServiceServer ask_incremental_finish_service_;

		// KnowledgeBase
		KnowledgeBasePtr kb_;

		// Mutex to protect query_results_
		std::mutex query_mutex_;
		// Counter to assign unique query IDs
		uint32_t next_query_id_ = 1;
		// Stores query results for each query ID
		std::map<uint32_t, std::shared_ptr<TokenQueue>> query_results_;

	public:

		/**
		 * Constructor
		 * @param ptree the property tree containing the configuration
		 */
		explicit ROSInterface(const boost::property_tree::ptree &ptree);

		virtual ~ROSInterface();

		/**
		 * execute the AskAll action
		 * @param goal AskAllGoalConstPtr
		 */
		void executeAskAllCB(const AskAllGoalConstPtr &goal);

		/**
		 * execute the AskOne action
		 * @param goal AskOneGoalConstPtr
		 */
		void executeAskOneCB(const AskOneGoalConstPtr &goal);

		/**
		 * execute the AskIncremental action
		 * @param goal AskIncrementalGoalConstPtr
		 */
		void executeAskIncrementalCB(const AskIncrementalGoalConstPtr &goal);

		/**
		 * execute the AskIncrementalNextSolution action
		 * @param goal AskIncrementalNextSolutionGoalConstPtr
		 */
		void executeAskIncrementalNextSolutionCB(const AskIncrementalNextSolutionGoalConstPtr &goal);

		/**
		 * execute the Tell action
		 * @param goal TellGoalConstPtr
		 */
		void executeTellCB(const TellGoalConstPtr &goal);

		/**
		 * Handle the AskIncrementalFinish service
		 * @param req AskIncrementalFinish::Request
		 * @param res AskIncrementalFinish::Response
		 * @return true if the service was handled successfully
		 */
		bool handleAskIncrementalFinish(AskIncrementalFinish::Request &req,
										AskIncrementalFinish::Response &res);

		/**
		 * Translate a GraphQueryMessage into a map of key-value pairs
		 * that can be used by applyModality
		 *
		 * @param query A GraphQueryMessage
		 * @return Map of key-value pairs
		 */
		std::unordered_map<std::string, boost::any> translateGraphQueryMessage(const GraphQueryMessage &query);

		/**
		 * Translate a answer into a GraphAnswerMessage
		 *
		 * @param sharedPtr shared pointer to the answer
		 * @return The GraphAnswerMessage
		 */
		GraphAnswerMessage createGraphAnswer(std::shared_ptr<const AnswerYes> sharedPtr);
	};
}

#endif //KNOWROB_ROSINTERFACE_H
