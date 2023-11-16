/*********************************************************************
 * @file:   mpc_path_follower_ros.h
 * @author: Lianchuan Zhang
 * @date:   2019.01.16
 * @version:1.0.1
 * @brief:  interface between mpc and ros navigation
 *********************************************************************/
#pragma once
#include <vector>
#include <math.h>
#include <mpc_path_follower/mpc_path_follower.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <navfn/navfn.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "parameter_manager.h"
#include "std_msgs/Float32.h"

namespace mpc_path_follower {
    class MpcPathFollowerRos : public nav_core::BaseLocalPlanner{
    public:
        /**
         * @brief  Constructor for mpc path follower wrapper
         */
        MpcPathFollowerRos();
        /**
         * @brief  Destructor for the wrapper
         */
        ~MpcPathFollowerRos(){};

        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
        /**
         * @brief  Given the current position, orientation, and velocity of the robot,
         * compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid trajectory was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Set the plan that the controller is following
         * @param orig_global_plan The plan to pass to the controller
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

        /**
         * @brief  Check if the goal pose has been achieved
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

    private:

        inline void publishZeroVelocity(){
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = 0.0;
          vel_pub_.publish(cmd_vel);
        }

        /**
         * @brief evaluate a polynominal. y= a*x^3 + b*x^2+c*x+d;
         * @param coefficients and input
         * @return output of the polynominal
         */
        double polyeval(Eigen::VectorXd coeffs, double x);

        /**
         * @brief fit a polynominal, y= a*x^3 + b*x^2+c*x+d;
         * @param vector x, vector y and order
         * @return output coefficients
         */
        Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                                int order);

        bool mpcComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::Twist& cmd_vel );

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

        bool publishCte(const std::vector<geometry_msgs::PoseStamped> &path);

        float FindClosestDistance(const std::vector<geometry_msgs::PoseStamped> &path, const geometry_msgs::PoseStamped &current_location);

        float FindDistance(const geometry_msgs::PoseStamped &current_location, const geometry_msgs::PoseStamped &next_location);

        // for visualization, publishers of global and local plan
        ros::Publisher g_plan_pub_, vel_pub_, cost_pub_;

        ros::Publisher _pub_ref_path_odom, _pub_mpc_traj_vehicle, _pub_ref_path_baselink, _pub_mpc_traj_map, pub_cte_;

        costmap_2d::Costmap2DROS* costmap_ros_;

        geometry_msgs::PoseStamped current_pose_;

        bool initialized_;

        base_local_planner::LocalPlannerUtil planner_util_;

        base_local_planner::OdometryHelperRos odom_helper_;

        std::string odom_topic_;

        std::vector<geometry_msgs::PoseStamped> global_plan_;

        MPC_Path_Follower mpc_solver_;

        bool _is_close_enough;

        ParameterManager params_;
    };
};
