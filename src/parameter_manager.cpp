

#include "parameter_manager.h"

namespace mpc_path_follower
{

  void ParameterManager::loadRosParamFromNodeHandle(const ros::NodeHandle &nh)
  {

    nh.getParam("odom_topic", odom_topic);
    nh.getParam("map_frame", map_frame);

    nh.getParam("planning_frequency", planning_frequency);
    nh.getParam("max_linear_velocity", max_linear_velocity);
    nh.getParam("min_linear_velocity", min_linear_velocity);

    nh.getParam("max_steering_angle", max_steering_angle);

    nh.getParam("max_angular_acceleration", max_angular_acceleration);
    nh.getParam("min_angular_acceleration", min_angular_acceleration);
    nh.getParam("max_linear_acceleration", max_linear_acceleration);
    nh.getParam("min_linear_acceleration", min_linear_acceleration);

    nh.getParam("vehicle_Lf", vehicle_Lf);
    nh.getParam("predicted_length", predicted_length);

    nh.getParam("evaluate_path", evaluate_path);

    nh.getParam("cte_weight", cte_weight);
    nh.getParam("epsi_weight", epsi_weight);
    nh.getParam("v_weight", v_weight);
    nh.getParam("delta_weight", delta_weight);
    nh.getParam("a_weight", a_weight);
    nh.getParam("delta_gap_weight", delta_gap_weight);
    nh.getParam("a_gap_weight", a_gap_weight);
    nh.getParam("ref_velocity", ref_velocity);
  }

} // namespace mpc_path_follower
