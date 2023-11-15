/**
 * @file path_evaluator.cpp
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.3
 * @date 2021-12-17
 *
 * @copyright Copyright (c) 2021
 *
 **/
#include "path_evaluator.h"

namespace mpc_path_follower
{

    void PathEvaluator::CallbackCmd(const geometry_msgs::Twist::ConstPtr &cmd, const std::string &topic_name)
    {
        linear_velocity_vec_.emplace_back(cmd->linear.x);
        steering_angle_vec_.emplace_back(cmd->angular.z);
        // DLOG(INFO) << "set speed";
    }

    void PathEvaluator::CalculateCost(const std_msgs::Float32::ConstPtr &cost, const std::string &topic_name)
    {
        cost_vec_.emplace_back(cost->data);
    }

    void PathEvaluator::Plot()
    {
        matplotlibcpp::ion();
        matplotlibcpp::clf();
        std::vector<std::string> title_vec = {"steering angle", "linear velocity", "cost"};
        std::vector<float> vec;
        for (size_t i = 0; i < title_vec.size(); i++)
        {
            matplotlibcpp::subplot(2, 3, i + 1);

            if (title_vec[i] == "steering angle")
            {
                vec = steering_angle_vec_;
            }
            if (title_vec[i] == "linear velocity")
            {
                vec = linear_velocity_vec_;
            }
            // if (title_vec[i] == "jerk")
            // {
            //     vec = jerk_vec_;
            // }
            if (title_vec[i] == "cost")
            {
                vec = cost_vec_;
            }

            matplotlibcpp::plot(vec, {{"label", "raw path"}});

            matplotlibcpp::legend({{"loc", "upper right"}});
            // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;

            matplotlibcpp::title(title_vec[i]);
            matplotlibcpp::ylabel(title_vec[i]);
            // matplotlibcpp::ylim(0, 1);
            matplotlibcpp::grid(true);
        }

        matplotlibcpp::pause(0.1);
    }

    void PathEvaluator::EvaluatePath()
    {
        // DLOG(INFO) << "in EvaluatePath:";
        // CalculateCurvature();
        // CalculateSmoothness();
        Plot();
    }
}
