/*********************************************************************
 * @file:   MpcPathFollowerRos.cpp
 * @author: Lianchuan Zhang
 * @date:   2019.01.16
 * @version:1.0.1
 * @brief:  interface between mpc and ros navigation
 *********************************************************************/

#include <mpc_path_follower/mpc_path_follower_ros.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(mpc_path_follower::MpcPathFollowerRos, nav_core::BaseLocalPlanner)

namespace mpc_path_follower {
    MpcPathFollowerRos::MpcPathFollowerRos() : initialized_(false),
                                               odom_helper_("odom"), _is_close_enough(false)
    {
        std::string log_dir = "/home/jialiang/Code/thesis_ws/src/mpc_path_follower_ros/log/mpc_local_planner_";
        for (int severity = 0; severity < google::NUM_SEVERITIES; ++severity)
        {
            google::SetLogDestination(severity, log_dir.c_str());
            google::SetLogSymlink(severity, log_dir.c_str());
        }
        google::InitGoogleLogging("mpc_local_planner");

        google::InstallFailureSignalHandler();

        google::EnableLogCleaner(5);
        FLAGS_alsologtostderr = 1;
        DLOG(INFO) << "creating mpc_local_planner planner";
    }

    void MpcPathFollowerRos::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            // DLOG(INFO) << "name is " << name;
            // ros::NodeHandle private_nh("/" + name);
            ros::NodeHandle nh("~/" + name);

            // load parameter:
            params_.loadRosParamFromNodeHandle(nh);

            // DLOG(INFO) << "planning frequency is " << params_.planning_frequency;
            mpc_solver_.initialize(params_.predicted_length, params_.vehicle_Lf, params_.planning_frequency, params_.max_steering_angle, params_.min_linear_acceleration, params_.max_linear_acceleration, params_.cte_weight, params_.epsi_weight, params_.v_weight, params_.delta_weight, params_.a_weight, params_.delta_gap_weight, params_.a_gap_weight, params_.ref_velocity);

            _pub_ref_path_odom = nh.advertise<nav_msgs::Path>("/mpc_reference_path_odom", 1);
            // _pub_ref_path_baselink = nh.advertise<nav_msgs::Path>("/mpc_reference_path_baselink", 1);

            _pub_mpc_traj_vehicle = nh.advertise<nav_msgs::Path>("/_pub_mpc_traj_vehicle", 1); // MPC trajectory output

            _pub_mpc_traj_map = nh.advertise<nav_msgs::Path>("/_pub_mpc_traj_map", 1); // MPC trajectory output

            pub_cte_ = nh.advertise<std_msgs::Float32>("cte", 1);

            cost_pub_ = nh.advertise<std_msgs::Float32>("/cost", 1);

            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);

            // make sure to update the costmap we'll use for this cycle
            costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
            planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
            if (nh.getParam("odom_topic", odom_topic_))
            {
                odom_helper_.setOdomTopic( odom_topic_ );
                // DLOG(INFO) << "odom topic set.";
            }
            else
            {
                odom_helper_.setOdomTopic("/odom");
                // DLOG(INFO) << "odom topic set.";
            }

            initialized_ = true;
            DLOG(INFO) << "initialized.";
        }
        else
        {
            DLOG(INFO) << "This planner has already been initialized, doing nothing.";
            // ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool MpcPathFollowerRos::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            DLOG(ERROR) << "Could not get robot pose";
            // ROS_ERROR("Could not get robot pose");
            return false;
        }
        DLOG(INFO) << "current_pose_ is " << current_pose_.pose.position.x << " " << current_pose_.pose.position.y;
        std::vector<geometry_msgs::PoseStamped> transformed_plan;

        if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
        {
            DLOG(ERROR) << "MPC Could not get local plan";
            // ROS_ERROR("MPC Could not get local plan");
            return false;
        }
        // if the global plan passed in is empty... we won't do anything
        if (transformed_plan.empty())
        {
            DLOG(WARNING) << "mpc_local_planner: Received an empty transformed plan.";
            // ROS_WARN_NAMED("mpc_local_planner", "Received an empty transformed plan.");
            return false;
        }
        // DLOG(INFO) << "mpc_local_planner: Received a transformed plan with " << transformed_plan.size() << " points.";
        // ROS_FATAL_NAMED("mpc_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

        if (isGoalReached())
        {
            // publish an empty plan because we've reached our goal position
            publishZeroVelocity();
            DLOG(INFO) << "publish zero velocity!!!";
            return true;
        }
        else
        {
            bool isOk = mpcComputeVelocityCommands(transformed_plan, cmd_vel);
            if (isOk)
            {
                // publishGlobalPlan(transformed_plan);
            }
            else
            {
                DLOG(INFO) << "mpc_local_planner: mpc planner failed to produce path.";
                // ROS_WARN_NAMED("mpc_local_planner", "mpc planner failed to produce path.");
                // std::vector<geometry_msgs::PoseStamped> empty_plan;
                // publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
    }

    bool MpcPathFollowerRos::mpcComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped> &path, geometry_msgs::Twist &cmd_vel){

        geometry_msgs::PoseStamped robot_vel;
        odom_helper_.getRobotVel(robot_vel);
        Eigen::Vector3f velocity(robot_vel.pose.position.x,
                                 robot_vel.pose.position.y, 0);
        float current_speed = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
        DLOG(INFO) << "speed x value is " << velocity[0] << " speed y value is " << velocity[1];
        // Display the MPC reference trajectory in odom coordinate
        nav_msgs::Path _mpc_ref_traj;
        _mpc_ref_traj.header.frame_id = "odom";
        _mpc_ref_traj.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped tempPose;
        tempPose.header = _mpc_ref_traj.header;
        for(int i = 0; i < path.size(); i++)
        {
            tempPose.pose = path.at(i).pose;
            // DLOG(INFO) << "local path is " << tempPose.pose.position.x << " " << tempPose.pose.position.y;
            _mpc_ref_traj.poses.push_back(tempPose);
        }
        _pub_ref_path_odom.publish(_mpc_ref_traj);
        // current vehicle position
        double px = current_pose_.pose.position.x;
        double py = current_pose_.pose.position.y;

        publishCte(path);
        // DLOG(INFO) << "px is " << px << " py is " << py;
        // DLOG(INFO) << "current_pose_ is " << current_pose_.pose.position.x << " " << current_pose_.pose.position.y;
        // current vehicle orientation angle
        double psi = tf::getYaw(current_pose_.pose.orientation);
        DLOG_IF(FATAL, std::isnan(psi)) << "psi is nan!!!!";
        // Waypoints related parameters
        // double cospsi = cos(psi);
        // double sinpsi = sin(psi);
        // Convert to the vehicle coordinate system
        // std::vector<double> waypoints_x, waypoints_y;
        Eigen::VectorXd waypoints_x(path.size());
        Eigen::VectorXd waypoints_y(path.size());
        // Display the MPC reference trajectory in odom coordinate
        // nav_msgs::Path _vehicle_ref_traj;
        // _vehicle_ref_traj.header.frame_id = "base_link"; // points in car coordinate
        // _vehicle_ref_traj.header.stamp = ros::Time::now();
        // tempPose.header = _vehicle_ref_traj.header;
        for(int i = 0; i < path.size(); i++)
        {
            double dx = path.at(i).pose.position.x - px;
            double dy = path.at(i).pose.position.y - py;
            waypoints_x[i] = (dx * std::cos(-psi) - dy * std::sin(-psi));
            waypoints_y[i] = (dy * std::cos(-psi) + dx * std::sin(-psi));
            // tempPose.pose.position.x = dx * cospsi + dy * sinpsi;
            // tempPose.pose.position.y = dy * cospsi - dx * sinpsi;
            // _vehicle_ref_traj.poses.push_back(tempPose);
            // DLOG(INFO) << "local path at vehicle frame is " << tempPose.pose.position.x << " " << tempPose.pose.position.y;
        }
        // _pub_ref_path_baselink.publish(_vehicle_ref_traj);
        int size_of_path = waypoints_x.size();
        if (size_of_path <= 6)
        {
            _is_close_enough = true;
            return true;
        }
        // san ci duo xiang shi ni he.
        double *ptrx = &waypoints_x[0];
        double *ptry = &waypoints_y[0];
        Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
        Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
        auto coeffs = polyfit(waypoints_x, waypoints_y, 3);
        /* The cross track error is calculated by evaluating at polynomial at x, f(x)
        and subtracting y.
        double cte = polyeval(coeffs, x) - y;
        Due to the sign starting at 0, the orientation error is -f'(x).
        derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
        double epsi = psi - atan(coeffs[1]);*/
        // calculate cte, cross-track error.
        // TODO change to cross-track error
        double cte = polyeval(coeffs, 0);
        // and epsi,  orientation error.
        // TODO change to orientation error
        // double epsi = atan(coeffs[1]);
        double epsi = -atan(coeffs[1]);
        // DLOG(INFO) << "psi is " << psi << " path size is " << path.size() << " waypoints x size is " << waypoints_x.size() << " cte is" << cte << " epsi is " << epsi;
        // for (int i = 0; i < coeffs.size(); i++)
        // {
        //     DLOG(INFO) << "element in coeffs is " << coeffs[i];
        // }

        // state: x,y,vehicle orientation angle, velocity,cross-track error. orientation error.
        Eigen::VectorXd state(6);
        state << 0, 0, 0, current_speed, cte, epsi;
        std::vector<double> vars;
        vars = mpc_solver_.solve(state, coeffs);
        auto cost = vars.back();
        std_msgs::Float32 cost_msg;
        cost_msg.data = cost;
        // DLOG(INFO) << "cost is " << cost_msg.data;
        cost_pub_.publish(cost_msg);
        // DLOG(INFO) << "vars size is " << vars.size();
        if (vars.size() < 2)
        {
            return false;
        }
        std::vector<double> mpc_x_vals, mpc_y_vals;
        // why divided 2?see solution structure
        for (int i = 2; i < vars.size(); i++)
        {
            // skip cost
            if (i == (vars.size() - 1))
            {
                continue;
            }

            if (i % 2 == 0)
            {
                mpc_x_vals.push_back(vars[i]);
            }
            else
            {
                mpc_y_vals.push_back(vars[i]);
            }
        }
        //  Display the MPC predicted trajectory
        nav_msgs::Path _mpc_predi_traj;
        _mpc_predi_traj.header.frame_id = "base_link"; // points in car coordinate
        _mpc_predi_traj.header.stamp = ros::Time::now();
        tempPose.header = _mpc_predi_traj.header;
        // DLOG(INFO) << "mpc_x_vals size is " << mpc_x_vals.size() << " mpc_y_vals size is " << mpc_y_vals.size();
        for (int i = 2; i < mpc_x_vals.size() - 3; i++)
        {
            tempPose.pose.position.x = mpc_x_vals[i];
            tempPose.pose.position.y = mpc_y_vals[i];
            tempPose.pose.orientation.w = 1.0;
            _mpc_predi_traj.poses.push_back(tempPose);
            // DLOG(INFO) << "_mpc_predi_traj base_link is " << tempPose.pose.position.x << " " << tempPose.pose.position.y;
        }
        _pub_mpc_traj_vehicle.publish(_mpc_predi_traj);

        // TODO transfer this trajectory to map frame
        //  Display the MPC predicted trajectory
        nav_msgs::Path _mpc_predi_traj_map;
        _mpc_predi_traj_map.header.frame_id = "map"; // points in map coordinate
        _mpc_predi_traj_map.header.stamp = ros::Time::now();
        tempPose.header = _mpc_predi_traj_map.header;
        for (int i = 2; i < mpc_x_vals.size() - 3; i++)
        {
            tempPose.pose.position.x = mpc_x_vals[i] + px;
            tempPose.pose.position.y = mpc_y_vals[i] + py;
            tempPose.pose.orientation.w = 1.0;
            _mpc_predi_traj_map.poses.push_back(tempPose);
            // DLOG(INFO) << "_mpc_predi_traj_map is " << tempPose.pose.position.x << " " << tempPose.pose.position.y;
        }
        _pub_mpc_traj_map.publish(_mpc_predi_traj_map);

        double steer_value = vars[0], throttle_value = vars[1];
        DLOG(INFO) << "Steer value is " << steer_value << " and throttle value is " << throttle_value;
        // ROS_INFO("Steer value and throttle value is, %lf , %lf", steer_value, throttle_value);
        cmd_vel.linear.x = velocity[0] + throttle_value * (1 / params_.planning_frequency) * std::cos(psi);
        cmd_vel.linear.y = velocity[1] + throttle_value * (1 / params_.planning_frequency) * std::sin(psi);
        // DLOG(INFO) << "speed x value is " << cmd_vel.linear.x << " current velocity speed x is " << velocity[0] << " throttle value is " << throttle_value << " planning freq is " << params_.planning_frequency << " current vehicle orientation is " << psi * 180 / 3.14 << "  std::cos(psi) is " << std::cos(psi) << " throttle_value * (1 / params_.planning_frequency) is " << throttle_value * (1 / params_.planning_frequency) << " throttle_value * (1 / params_.planning_frequency) * std::cos(psi) is " << throttle_value * (1 / params_.planning_frequency) * std::cos(psi);
        // double radius;
        // if (fabs(tan(steer_value)) <= 1e-2)
        // {
        //     radius = 1e5;
        // }
        // else
        // {
        //     // TODO why 0.5??
        //     radius = 0.5 / tan(steer_value);
        // }
        // angular velocity = linear velocity/vehicle length* tan(steering angle)
        float next_vehicle_velocity = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);

        cmd_vel.angular.z = next_vehicle_velocity / params_.vehicle_Lf * std::tan(steer_value);
        // cmd_vel.angular.z = 0;
        // cmd_vel.angular.z = std::max(-1.0, std::min(1.0, (cmd_vel.linear.x / radius)));
        // cmd_vel.linear.x = std::min(0.2, cmd_vel.linear.x);
        DLOG(INFO) << "speed x value is " << cmd_vel.linear.x << " speed y value is " << cmd_vel.linear.y << " and z value is " << cmd_vel.angular.z;
        // ROS_INFO("v value and z value is, %lf , %lf", cmd_vel.linear.x, cmd_vel.angular.z);

        return true;
    }

    bool MpcPathFollowerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if(!initialized_){
            DLOG(INFO) << "Planner have not been initialized, please call initialize() first";
            // ROS_ERROR("Planner utils have not been initialized, please call initialize() first");
            return false;
        }
        // store the global plan
        global_plan_.clear();

        global_plan_ = orig_global_plan;

        // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
        // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

        _is_close_enough = false;
        planner_util_.setPlan(global_plan_);
        return true;
    }

    bool MpcPathFollowerRos::isGoalReached(){
        if (!initialized_)
        {
            DLOG(INFO) << "Planner have not been initialized, please call initialize() first";
            // ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (!costmap_ros_->getRobotPose(current_pose_))
        {
            DLOG(WARNING) << "Could not get robot pose";
            // ROS_ERROR("Could not get robot pose");
            return false;
        }

        if (_is_close_enough)
        {
            DLOG(INFO) << "Goal reached";
            // ROS_INFO("Goal reached");
            return true;
        }
        else
        {
            return false;
        }
    }

    double MpcPathFollowerRos::polyeval(Eigen::VectorXd coeffs, double x){
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }

    Eigen::VectorXd MpcPathFollowerRos::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                                                   int order){
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
        {
            A(i, 0) = 1.0;
        }

        for (int j = 0; j < xvals.size(); j++)
        {
            for (int i = 0; i < order; i++)
            {
                A(j, i + 1) = A(j, i) * xvals(j);
                // DLOG(INFO) << j << "th element is " << xvals(j);
            }
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    void MpcPathFollowerRos::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
    {
        base_local_planner::publishPlan(path, g_plan_pub_);
    }

    bool MpcPathFollowerRos::publishCte(const std::vector<geometry_msgs::PoseStamped> &path)
    {
        if (path.size() <= 0)
        {
            DLOG(INFO) << "path size is zero!!!";
            return false;
        }
        float cte = FindClosestDistance(path, current_pose_);
        std_msgs::Float32 cte_msg;
        cte_msg.data = cte;
        // DLOG(INFO) << "cte is " << cte_msg.data;
        pub_cte_.publish(cte_msg);
        return true;
    }

    float MpcPathFollowerRos::FindClosestDistance(const std::vector<geometry_msgs::PoseStamped> &path, const geometry_msgs::PoseStamped &current_location)
    {
        float min_distance = 1000000;
        float distance;
        for (size_t i = 0; i < path.size(); i++)
        {
            distance = FindDistance(path[i], current_location);
            if (distance < min_distance)
            {
                min_distance = distance;
            }
        }
        return min_distance;
    }

    float MpcPathFollowerRos::FindDistance(const geometry_msgs::PoseStamped &current_location, const geometry_msgs::PoseStamped &next_location)
    {
        float distance, dx, dy;

        dx = current_location.pose.position.x - next_location.pose.position.x;
        dy = current_location.pose.position.y - next_location.pose.position.y;
        distance = std::sqrt(dx * dx + dy * dy);
        return distance;
    }
};
