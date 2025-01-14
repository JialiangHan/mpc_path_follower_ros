/**
 * @file path_evaluator.h
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.1
 * @date 2021-12-07
 *
 * @copyright Copyright (c) 2021
 *
 **/
#pragma once
#include <vector>
#include <string>
#include "ros/ros.h"
#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>
#include "matplotlibcpp.h"
#include <algorithm>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "glog/logging.h"
#include "gflags/gflags.h"

namespace mpc_path_follower
{
    class PathEvaluator
    {
    public:
        PathEvaluator(){};
        PathEvaluator(const std::string &cmd_topic, const std::string &cost_topic, const std::string &cte_topic)
        {
            // sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic, 1, boost::bind(&PathEvaluator::CallbackPath, this, _1, path_topic));
            sub_cmd_ = nh_.subscribe<geometry_msgs::Twist>(cmd_topic, 1, boost::bind(&PathEvaluator::CallbackCmd, this, _1, cmd_topic));
            sub_cte_ = nh_.subscribe<std_msgs::Float32>(cte_topic, 1, boost::bind(&PathEvaluator::CalculateCte, this, _1, cte_topic));
            sub_cost_ = nh_.subscribe<std_msgs::Float32>(cost_topic, 1, boost::bind(&PathEvaluator::CalculateCost, this, _1, cost_topic));
        };

        // void CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name);

        void CallbackCmd(const geometry_msgs::Twist::ConstPtr &cmd, const std::string &topic_name);

        // void CallbackJerk(const std_msgs::Float32::ConstPtr &jerk, const std::string &topic_name);

        void CalculateCost(const std_msgs::Float32::ConstPtr &cost, const std::string &topic_name);

        void CalculateCte(const std_msgs::Float32::ConstPtr &cte, const std::string &topic_name);

        void EvaluatePath();
        /**
         * @brief calculate curvature for the path
         *
         * @param path got from planner
         * @return std::vector<float>
         */
        // int CalculateCurvature();

        // int CalculateSmoothness();

        void Plot();

    private:
        ros::NodeHandle nh_;

        // ros::Subscriber sub_path_;

        ros::Subscriber sub_cmd_;

        ros::Subscriber sub_cte_;

        ros::Subscriber sub_cost_;

        // std::vector<Eigen::Vector3f> path_;

        std::vector<float> cte_vec_;

        // std::vector<float> smoothness_vec_;

        std::vector<float> steering_angle_vec_;

        std::vector<float> linear_velocity_vec_;

        std::vector<float> cost_vec_;
    };
}
