/**
 * @file node_path_evaluator.cpp
 * @author Jialiang Han
 * @brief just plot path evaluator
 * @version 0.1
 * @date 2021-12-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <cstring>
#include "path_evaluator.h"
// namespace mpc_path_follower
// {
int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);

    google::ParseCommandLineFlags(&argc, &argv, true);

    google::InstallFailureSignalHandler();

    google::EnableLogCleaner(5);

    ros::init(argc, argv, "path_evaluator");
    std::string cmd_topic = "cmd_vel";
    std::string cost_topic = "cost";
    std::string cte_topic = "cte";
    mpc_path_follower::PathEvaluator path_evaluator(cmd_topic, cost_topic, cte_topic);

    while (ros::ok())
    {
        path_evaluator.EvaluatePath();
        ros::spinOnce();
    }

    return 0;
}
// }