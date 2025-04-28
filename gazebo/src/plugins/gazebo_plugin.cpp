#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
  class GazeboPlugin : public ModelPlugin
  {
  public:
    GazeboPlugin()
      : logger_(rclcpp::get_logger("gazebo_plugin")) // Properly initialize the logger
    {
      // Initialize ROS2 if it hasn't been initialized yet
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }

      RCLCPP_INFO(logger_, "GazeboPlugin loaded!");
    }

    ~GazeboPlugin()
    {
      // Shutdown ROS2 if it is still running
      if (rclcpp::ok())
      {
        rclcpp::shutdown();
      }
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      this->model = model;
      RCLCPP_INFO(logger_, "Plugin attached to model: %s", model->GetName().c_str());
    }

  private:
    physics::ModelPtr model;
    rclcpp::Logger logger_; // Logger must be initialized in the initializer list
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(GazeboPlugin)
}