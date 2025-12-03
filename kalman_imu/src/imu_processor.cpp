#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <kalman_interfaces/msg/imu_data.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <vector>

class IMUProcessorNode : public rclcpp::Node
{
public:
  IMUProcessorNode()
  : Node("imu_processor_node"),
    calibration_done_(false),
    calibration_sample_count_(0)
  {
    // Declare parameters
    this->declare_parameter("imu.topic_name_sub", "/imu_telem");
    this->declare_parameter("imu.topic_name_pub", "/imu_processed");
    this->declare_parameter("imu.accel_scale", 16384.0);      // LSB/g for ±2g
    this->declare_parameter("imu.gyro_scale", 131.0);         // LSB/°/s for ±250°/s
    this->declare_parameter("imu.gravity_accel", 9.81);       // m/s²
    this->declare_parameter("imu.complementary_alpha", 0.96); // Complementary filter coefficient
    this->declare_parameter("imu.calibration_samples", 1000); // Number of samples for calibration
    this->declare_parameter("imu.auto_calibrate", true);      // Auto-calibrate on startup

    // Get parameters
    accel_scale_ = this->get_parameter("imu.accel_scale").as_double();
    gyro_scale_ = this->get_parameter("imu.gyro_scale").as_double();
    gravity_ = this->get_parameter("imu.gravity_accel").as_double();
    alpha_ = this->get_parameter("imu.complementary_alpha").as_double();
    calibration_samples_ = this->get_parameter("imu.calibration_samples").as_int();
    auto_calibrate_ = this->get_parameter("imu.auto_calibrate").as_bool();
    
    deg_to_rad_ = M_PI / 180.0;
    rad_to_deg_ = 180.0 / M_PI;

    // Initialize Euler angles
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
    
    last_time_ = this->now();

    // Initialize calibration accumulators
    gyro_sum_x_ = 0.0;
    gyro_sum_y_ = 0.0;
    gyro_sum_z_ = 0.0;
    accel_sum_x_ = 0.0;
    accel_sum_y_ = 0.0;
    accel_sum_z_ = 0.0;

    // Try to load existing calibration
    bool calibration_loaded = load_calibration();

    // Create subscriber
    imu_sub_ = this->create_subscription<kalman_interfaces::msg::ImuData>(
      this->get_parameter("imu.topic_name_sub").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&IMUProcessorNode::imu_callback, this, std::placeholders::_1));

    // Create publisher
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      this->get_parameter("imu.topic_name_pub").as_string(),
      10);

    RCLCPP_INFO(this->get_logger(), "====================================");
    RCLCPP_INFO(this->get_logger(), "IMU Processor Node started");
    RCLCPP_INFO(this->get_logger(), "====================================");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", 
                this->get_parameter("imu.topic_name_sub").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", 
                this->get_parameter("imu.topic_name_pub").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Complementary filter alpha: %.3f", alpha_);

    if (auto_calibrate_) {
      RCLCPP_WARN(this->get_logger(), "====================================");
      RCLCPP_WARN(this->get_logger(), "🔧 Auto-calibration will start...");
      RCLCPP_WARN(this->get_logger(), "⚠️  Keep the sensor STILL and FLAT!");
      RCLCPP_WARN(this->get_logger(), "Collecting %d samples...", calibration_samples_);
      RCLCPP_WARN(this->get_logger(), "====================================");
    } else if (calibration_loaded) {
      calibration_done_ = true;
      RCLCPP_INFO(this->get_logger(), "✅ Using loaded calibration data");
      RCLCPP_INFO(this->get_logger(), "Accel offset (g): [%.4f, %.4f, %.4f]", 
                  accel_offset_x_, accel_offset_y_, accel_offset_z_);
      RCLCPP_INFO(this->get_logger(), "Gyro offset (°/s): [%.4f, %.4f, %.4f]", 
                  gyro_offset_x_, gyro_offset_y_, gyro_offset_z_);
    }
  }

private:
  bool load_calibration()
  {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("kalman_imu");
    std::string calib_file = package_share_dir + "/config/calibration.yaml";

    // Initialize offsets to zero
    accel_offset_x_ = 0.0;
    accel_offset_y_ = 0.0;
    accel_offset_z_ = 0.0;
    gyro_offset_x_ = 0.0;
    gyro_offset_y_ = 0.0;
    gyro_offset_z_ = 0.0;

    std::ifstream file(calib_file);
    if (!file.good()) {
      RCLCPP_WARN(this->get_logger(), "Calibration file not found: %s", calib_file.c_str());
      return false;
    }

    try {
      YAML::Node config = YAML::LoadFile(calib_file);

      if (config["accelerometer"] && config["accelerometer"]["offset"]) {
        accel_offset_x_ = config["accelerometer"]["offset"]["x"].as<double>();
        accel_offset_y_ = config["accelerometer"]["offset"]["y"].as<double>();
        accel_offset_z_ = config["accelerometer"]["offset"]["z"].as<double>();
      }

      if (config["gyroscope"] && config["gyroscope"]["offset"]) {
        gyro_offset_x_ = config["gyroscope"]["offset"]["x"].as<double>();
        gyro_offset_y_ = config["gyroscope"]["offset"]["y"].as<double>();
        gyro_offset_z_ = config["gyroscope"]["offset"]["z"].as<double>();
      }

      RCLCPP_INFO(this->get_logger(), "Calibration loaded successfully from: %s", calib_file.c_str());
      return true;

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading calibration: %s", e.what());
      return false;
    }
  }

  void save_calibration()
  {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("kalman_imu");
    std::string calib_file = package_share_dir + "/config/calibration.yaml";

    try {
      YAML::Emitter out;
      out << YAML::BeginMap;
      
      out << YAML::Key << "accelerometer";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "offset";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << accel_offset_x_;
      out << YAML::Key << "y" << YAML::Value << accel_offset_y_;
      out << YAML::Key << "z" << YAML::Value << accel_offset_z_;
      out << YAML::EndMap;
      out << YAML::EndMap;
      
      out << YAML::Key << "gyroscope";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "offset";
      out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << gyro_offset_x_;
      out << YAML::Key << "y" << YAML::Value << gyro_offset_y_;
      out << YAML::Key << "z" << YAML::Value << gyro_offset_z_;
      out << YAML::EndMap;
      out << YAML::EndMap;
      
      out << YAML::EndMap;

      std::ofstream fout(calib_file);
      fout << out.c_str();
      fout.close();

      RCLCPP_INFO(this->get_logger(), "✅ Calibration saved to: %s", calib_file.c_str());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error saving calibration: %s", e.what());
    }
  }

  void perform_calibration(const kalman_interfaces::msg::ImuData & imu_data_msg)
  {
    // Accumulate samples
    gyro_sum_x_ += imu_data_msg.gyro_x;
    gyro_sum_y_ += imu_data_msg.gyro_y;
    gyro_sum_z_ += imu_data_msg.gyro_z;
    
    accel_sum_x_ += imu_data_msg.accel_x;
    accel_sum_y_ += imu_data_msg.accel_y;
    accel_sum_z_ += imu_data_msg.accel_z;
    
    calibration_sample_count_++;

    // Progress indicator (every 100 samples)
    if (calibration_sample_count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Calibration progress: %d/%d samples", 
                  calibration_sample_count_, calibration_samples_);
    }

    // Check if calibration is complete
    if (calibration_sample_count_ >= calibration_samples_) {
      // Calculate gyroscope offsets (average in °/s)
      gyro_offset_x_ = (gyro_sum_x_ / calibration_sample_count_) / gyro_scale_;
      gyro_offset_y_ = (gyro_sum_y_ / calibration_sample_count_) / gyro_scale_;
      gyro_offset_z_ = (gyro_sum_z_ / calibration_sample_count_) / gyro_scale_;

      // Calculate accelerometer offsets (in g)
      // X and Y should be 0g when flat
      accel_offset_x_ = (accel_sum_x_ / calibration_sample_count_) / accel_scale_;
      accel_offset_y_ = (accel_sum_y_ / calibration_sample_count_) / accel_scale_;
      // Z should be 1g when flat (gravity), so offset is (measured - 1.0)
      accel_offset_z_ = ((accel_sum_z_ / calibration_sample_count_) / accel_scale_) - 1.0;

      calibration_done_ = true;

      RCLCPP_INFO(this->get_logger(), "====================================");
      RCLCPP_INFO(this->get_logger(), "✅ Calibration Complete!");
      RCLCPP_INFO(this->get_logger(), "====================================");
      RCLCPP_INFO(this->get_logger(), "Gyroscope Offsets (°/s):");
      RCLCPP_INFO(this->get_logger(), "  X: %.4f  Y: %.4f  Z: %.4f", 
                  gyro_offset_x_, gyro_offset_y_, gyro_offset_z_);
      RCLCPP_INFO(this->get_logger(), "Accelerometer Offsets (g):");
      RCLCPP_INFO(this->get_logger(), "  X: %.4f  Y: %.4f  Z: %.4f", 
                  accel_offset_x_, accel_offset_y_, accel_offset_z_);
      RCLCPP_INFO(this->get_logger(), "====================================");

      // Save calibration to file
      save_calibration();

      RCLCPP_INFO(this->get_logger(), "Starting IMU processing...");
    }
  }

  void imu_callback(const kalman_interfaces::msg::ImuData & imu_data_msg)
  {
    // If auto-calibration is enabled and not done, perform calibration
    if (auto_calibrate_ && !calibration_done_) {
      perform_calibration(imu_data_msg);
      return;
    }

    // Calculate time delta
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Skip first iteration (dt would be unreliable)
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    // Apply calibration offsets and convert to physical units
    double accel_x_g = (imu_data_msg.accel_x / accel_scale_) - accel_offset_x_;
    double accel_y_g = (imu_data_msg.accel_y / accel_scale_) - accel_offset_y_;
    double accel_z_g = (imu_data_msg.accel_z / accel_scale_) - accel_offset_z_;

    double gyro_x_dps = (imu_data_msg.gyro_x / gyro_scale_) - gyro_offset_x_;
    double gyro_y_dps = (imu_data_msg.gyro_y / gyro_scale_) - gyro_offset_y_;
    double gyro_z_dps = (imu_data_msg.gyro_z / gyro_scale_) - gyro_offset_z_;

    // Calculate angles from accelerometer
    double accel_roll = std::atan2(accel_y_g, accel_z_g) * rad_to_deg_;
    double accel_pitch = std::atan2(-accel_x_g, 
                                     std::sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * rad_to_deg_;

    // Integrate gyroscope rates
    double gyro_roll_delta = gyro_x_dps * dt;
    double gyro_pitch_delta = gyro_y_dps * dt;
    double gyro_yaw_delta = gyro_z_dps * dt;

    // Complementary filter (fuse accelerometer + gyroscope)
    roll_ = alpha_ * (roll_ + gyro_roll_delta) + (1.0 - alpha_) * accel_roll;
    pitch_ = alpha_ * (pitch_ + gyro_pitch_delta) + (1.0 - alpha_) * accel_pitch;
    yaw_ += gyro_yaw_delta;

    // Normalize yaw to -180 to +180 degrees
    while (yaw_ > 180.0) yaw_ -= 360.0;
    while (yaw_ < -180.0) yaw_ += 360.0;

    // Apply coordinate system correction
    // Original: roll=X, pitch=Y, yaw=Z
    // Corrected: roll=-Y, pitch=X, yaw=Z
    double corrected_roll = -pitch_;   // roll should be (-pitch)
    double corrected_pitch = roll_;    // pitch should be (roll)
    double corrected_yaw = yaw_;       // yaw is correct

    // Create Imu message
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";

    // Convert Euler angles to quaternion
    tf2::Quaternion q;
    q.setRPY(corrected_roll * deg_to_rad_, 
             corrected_pitch * deg_to_rad_, 
             corrected_yaw * deg_to_rad_);

    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    // Set linear acceleration (m/s²)
    imu_msg.linear_acceleration.x = accel_x_g * gravity_;
    imu_msg.linear_acceleration.y = accel_y_g * gravity_;
    imu_msg.linear_acceleration.z = accel_z_g * gravity_;

    // Set angular velocity (rad/s)
    imu_msg.angular_velocity.x = gyro_x_dps * deg_to_rad_;
    imu_msg.angular_velocity.y = gyro_y_dps * deg_to_rad_;
    imu_msg.angular_velocity.z = gyro_z_dps * deg_to_rad_;

    // Publish message
    imu_pub_->publish(imu_msg);
    RCLCPP_INFO(this->get_logger(), 
                "Euler Angles - Roll: %.2f° Pitch: %.2f° Yaw: %.2f°",
                corrected_roll, corrected_pitch, corrected_yaw);
    // // Log info periodically (every 50 messages ~ 0.5s at 100Hz)
    // static int msg_count = 0;
    // if (++msg_count >= 50) {
    //   msg_count = 0;
    //   RCLCPP_INFO(this->get_logger(), 
    //               "Euler Angles - Roll: %.2f° Pitch: %.2f° Yaw: %.2f°",
    //               corrected_roll, corrected_pitch, corrected_yaw);
    // }
  }

  rclcpp::Subscription<kalman_interfaces::msg::ImuData>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Scaling parameters
  double accel_scale_;
  double gyro_scale_;
  double gravity_;
  double alpha_;
  double deg_to_rad_;
  double rad_to_deg_;

  // Calibration parameters
  int calibration_samples_;
  bool auto_calibrate_;
  bool calibration_done_;
  int calibration_sample_count_;

  // Calibration accumulators
  double gyro_sum_x_, gyro_sum_y_, gyro_sum_z_;
  double accel_sum_x_, accel_sum_y_, accel_sum_z_;

  // Calibration offsets
  double accel_offset_x_, accel_offset_y_, accel_offset_z_;
  double gyro_offset_x_, gyro_offset_y_, gyro_offset_z_;

  // Euler angles (degrees)
  double roll_, pitch_, yaw_;

  // Timing
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUProcessorNode>());
  rclcpp::shutdown();
  return 0;
}