#include <laser_scan_matcher/param_handler.h>

namespace scan_tools {

void ParamHandler::initialize(rclcpp::Node::SharedPtr node) {
  node_ = node;
}

void ParamHandler::registerCallback() {
  params_callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&ParamHandler::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ParamHandler::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param: parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      auto bool_param = bool_params_.find(param.get_name());
      if (bool_param != bool_params_.end()) {
        *bool_param->second = param.as_bool();
        RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %s", param.get_name().c_str(), (param.as_bool() ? "true": "false"));
      }
      else {
        auto int_param = int_params_.find(param.get_name());
        if (int_param != int_params_.end()) {
          *int_param->second = param.as_bool();
          RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %s", param.get_name().c_str(), (param.as_bool() ? "true": "false"));
        }
        RCLCPP_WARN(node_->get_logger(), "Unknown bool parameter updated: %s", param.get_name().c_str());
      }
    }
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      auto double_param = double_params_.find(param.get_name());
      if (double_param != double_params_.end()) {
        *double_param->second = param.as_double();
        RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %lf", param.get_name().c_str(), param.as_double());
      }
      else {
        RCLCPP_WARN(node_->get_logger(), "Unknown double parameter updated: %s", param.get_name().c_str());
      }
    }
    else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      auto int_param = int_params_.find(param.get_name());
      if (int_param != int_params_.end()) {
        *int_param->second = param.as_int();
        RCLCPP_INFO(node_->get_logger(), "Parameter updated: %s = %ld", param.get_name().c_str(), param.as_int());
      }
      else {
        RCLCPP_WARN(node_->get_logger(), "Unknown int parameter updated: %s", param.get_name().c_str());
      }
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Unsupported parameter type updated: %s (%s)", param.get_name().c_str(), param.get_type_name().c_str());
    }
  }

  return result;
}

void ParamHandler::register_param(
  int* param, const std::string& name, bool default_val, const std::string& description)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
  descriptor.description = description;
  descriptor.read_only = false;
  node_->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
  *param = node_->get_parameter(name).as_bool();
  int_params_[name] = param;
}

void ParamHandler::register_param(
  int* param, const std::string& name, int default_val, const std::string& description, int min, int max)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
  descriptor.description = description;
  descriptor.read_only = false;
  descriptor.integer_range.resize(1);
  descriptor.integer_range[0].from_value = min;
  descriptor.integer_range[0].to_value = max;
  descriptor.integer_range[0].step = 0;
  node_->declare_parameter(name, default_val, descriptor);
  *param = node_->get_parameter(name).as_int();
  int_params_[name] = param;
}

void ParamHandler::register_param(
  double* param, const std::string& name, double default_val, const std::string& description, double min, double max)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
  descriptor.description = description;
  descriptor.read_only = false;
  descriptor.floating_point_range.resize(1);
  descriptor.floating_point_range[0].from_value = min;
  descriptor.floating_point_range[0].to_value = max;
  descriptor.floating_point_range[0].step = 0;
  node_->declare_parameter(name, default_val, descriptor);
  *param = node_->get_parameter(name).as_double();
  double_params_[name] = param;
}

}  // namespace scan_tools
