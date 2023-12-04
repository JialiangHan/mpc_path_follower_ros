#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace mpc_path_follower
{

  /**
   * @class ParameterManager
   * @brief Config class for the teb_local_planner and its components.
   */
  class ParameterManager
  {
  public:
    std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
    std::string map_frame;  //!< Global planning frame

    double planning_frequency;
    float max_linear_velocity;
    float min_linear_velocity;
    double max_steering_angle;

      // speed limit

    float max_angular_acceleration;
    float min_angular_acceleration;
    float max_linear_acceleration;
    float min_linear_acceleration;

    // MPC parameter
    float vehicle_Lf;
    int predicted_length;
    bool evaluate_path;

    // weighting factor
    float cte_weight;
    float epsi_weight;
    float v_weight;
    float delta_weight;
    float a_weight;
    float delta_gap_weight;
    float a_gap_weight;
    float ref_velocity;

    ParameterManager()
    {
      odom_topic = "odom";
      map_frame = "odom";

      planning_frequency = 10;
      max_linear_velocity = 5;
      min_linear_velocity = 0;
      max_steering_angle = 0.5;
      max_angular_acceleration = 100;
      min_angular_acceleration = -100;
      max_linear_acceleration = 4;
      min_linear_acceleration = -4;

      vehicle_Lf = 0.325;
      predicted_length = 30;
      evaluate_path = false;
      cte_weight = 700;
      epsi_weight = 500;
      v_weight = 1;
      delta_weight = 10;
      a_weight = 10;
      delta_gap_weight = 10000000;
      a_gap_weight = 1000;
      ref_velocity = 40;
    }

    /**
     * @brief Load parameters from the ros param server.
     * @param nh const reference to the local ros::NodeHandle
     */
    void loadRosParamFromNodeHandle(const ros::NodeHandle &nh);
  };

} // namespace mpc_path_follower
