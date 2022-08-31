#pragma once

#include <rclcpp/rclcpp.hpp>

namespace scan_tools {

class ParamHandler {
public:
  ParamHandler() = default;
  ~ParamHandler() = default;

  void initialize(rclcpp::Node::SharedPtr node);

  void registerCallback();

  /**
   * Register a non-dynamic parameter and return it's value.
   *
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   *
   * @returns the value of the parameter.
   */
  template <class T>
  T param(const std::string& name, const T& default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = true;
    node_->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    return node_->get_parameter(name).get_value<T>();
  }

  /**
   * Register a dynamic parameter without any range constraint and populate
   * the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   */
  template <class T>
  void register_param(T* param, const std::string& name, const T& default_val, const std::string& description) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.description = description;
    descriptor.read_only = false;
    node_->declare_parameter(name, rclcpp::ParameterValue(default_val), descriptor);
    auto p = node_->get_parameter(name);
    *param = p.get_value<T>();

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      bool_params_[name] = static_cast<bool*>(static_cast<void*>(param));
    }
    else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      double_params_[name] = static_cast<double*>(static_cast<void*>(param));
    }
    else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      int_params_[name] = static_cast<int*>(static_cast<void*>(param));
    }
    else {
      RCLCPP_ERROR(node_->get_logger(), "Unsupported dynamic parameter type: %s for parameter: %s", p.get_type_name().c_str(), name.c_str());
    }
  }

  /**
   * Register a dynamic bool parameter stored in an integer variable and
   * populate the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   */
  void register_param(int* param, const std::string& name, bool default_val, const std::string& description);

  /**
   * Register a dynamic integer parameter with a range constraint and populate
   * the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   * @param[in] min          Min value
   * @param[in] max          Max value
   */
  void register_param(int* param, const std::string& name, int default_val, const std::string& description, int min, int max);

  /**
   * Register a dynamic double parameter with a range constraint and populate
   * the parameter variable with the current value.
   *
   * @param[out] param       Reference to parameter variable
   * @param[in] name         Parameter name
   * @param[in] default_val  Default parameter value
   * @param[in] description  Parameter description
   * @param[in] min          Min value
   * @param[in] max          Max value
   */
  void register_param(double* param, const std::string& name, double default_val, const std::string& description, double min, double max);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  std::unordered_map<std::string, bool*> bool_params_;
  std::unordered_map<std::string, double*> double_params_;
  std::unordered_map<std::string, int*> int_params_;
  std::unordered_map<std::string, std::string*> string_params_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
};

}  // scan_tools
