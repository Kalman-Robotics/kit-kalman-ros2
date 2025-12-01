#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <kalman_interfaces/msg/imu_data.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>

class IMUNode : public rclcpp::Node
{
public:
  IMUNode()
  : Node("imu_node")
  {
    // Declarar par�metros
    this->declare_parameter("imu.topic_name_sub", "/imu_telem");
    this->declare_parameter("imu.topic_name_pub", "/imu_raw");

    // Sensor scaling parameters (ICM6500: ±16g accel, ±2000°/s gyro)
    this->declare_parameter("imu.accel_scale", 2048.0);      // LSB/g
    this->declare_parameter("imu.gyro_scale", 16.384);       // LSB/°/s
    this->declare_parameter("imu.gravity_accel", 9.81);      // m/s²

    // Low-pass filter parameters (exponential smoothing)
    // alpha = 0.1 (strong smoothing, more lag)
    // alpha = 0.5 (medium smoothing)
    // alpha = 0.9 (light smoothing, less lag)
    this->declare_parameter("imu.filter_alpha", 0.15);       // Low-pass filter coefficient

    // Complementary filter parameter for orientation fusion
    // ALPHA closer to 1.0 = trust gyroscope more (fast response, drifts)
    // ALPHA closer to 0.0 = trust accelerometer more (stable, noisy)
    this->declare_parameter("imu.orientation_alpha", 0.98);  // Complementary filter coefficient
    // Whether to publish orientation (default: true)
    this->declare_parameter("imu.publish_orientation", true);
    
    // Crear suscriptor para /imu/data (kalman_interfaces::msg::ImuData)
    imu_sub_ = this->create_subscription<kalman_interfaces::msg::ImuData>(
      this->get_parameter("imu.topic_name_sub").as_string(),
      rclcpp::SensorDataQoS(),
      std::bind(&IMUNode::imu_callback, this, std::placeholders::_1));

    // Crear publicador para /imu (sensor_msgs::msg::Imu)
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      this->get_parameter("imu.topic_name_pub").as_string(),
      10);

    // Get scaling parameters
    accel_scale_ = this->get_parameter("imu.accel_scale").as_double();
    gyro_scale_ = this->get_parameter("imu.gyro_scale").as_double();
    gravity_ = this->get_parameter("imu.gravity_accel").as_double();

    // Get filter parameters
    filter_alpha_ = this->get_parameter("imu.filter_alpha").as_double();
    orientation_alpha_ = this->get_parameter("imu.orientation_alpha").as_double();
    publish_orientation_ = this->get_parameter("imu.publish_orientation").as_bool();

    // Conversion constant: degrees to radians
    deg_to_rad_ = M_PI / 180.0;

    // Initialize filter state (first measurement is treated as is)
    filter_initialized_ = false;
    orientation_initialized_ = false;

    // Initialize orientation angles (degrees)
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;

    // Initialize timestamp
    last_time_ = this->now();

    // Cargar parámetros de calibración desde YAML
    load_calibration();

    RCLCPP_INFO(this->get_logger(), "Nodo IMU iniciado");
    RCLCPP_INFO(this->get_logger(), "Suscrito a: %s", this->get_parameter("imu.topic_name_sub").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Publicando en: %s", this->get_parameter("imu.topic_name_pub").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "Accel scale: %.2f LSB/g, Gyro scale: %.4f LSB/°/s", accel_scale_, gyro_scale_);
    RCLCPP_INFO(this->get_logger(), "Accel offset: [%.2f, %.2f, %.2f] LSB", accel_offset_x_, accel_offset_y_, accel_offset_z_);
    RCLCPP_INFO(this->get_logger(), "Gyro offset: [%.2f, %.2f, %.2f] LSB", gyro_offset_x_, gyro_offset_y_, gyro_offset_z_);
    RCLCPP_INFO(this->get_logger(), "Low-pass filter alpha: %.3f (0.1=strong smoothing, 0.9=light smoothing)", filter_alpha_);
    RCLCPP_INFO(this->get_logger(), "Orientation filter alpha: %.3f (%.2f gyro / %.2f accel)", 
                orientation_alpha_, orientation_alpha_, 1.0 - orientation_alpha_);
    RCLCPP_INFO(this->get_logger(), "Publish orientation: %s", publish_orientation_ ? "true" : "false");
  }

private:
  void load_calibration()
  {
    // Obtener ruta del paquete
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("kalman_utils");
    std::string calib_file = package_share_dir + "/config/imu_calibration.yaml";

    // Inicializar offsets en cero
    accel_offset_x_ = 0.0;
    accel_offset_y_ = 0.0;
    accel_offset_z_ = 0.0;
    gyro_offset_x_ = 0.0;
    gyro_offset_y_ = 0.0;
    gyro_offset_z_ = 0.0;

    // Intentar cargar el archivo YAML
    std::ifstream file(calib_file);
    if (!file.good()) {
      RCLCPP_WARN(this->get_logger(), "Archivo de calibración no encontrado: %s", calib_file.c_str());
      RCLCPP_WARN(this->get_logger(), "Usando offsets por defecto (todos cero)");
      return;
    }

    try {
      YAML::Node config = YAML::LoadFile(calib_file);

      // Cargar offsets de aceleración
      if (config["accelerometer"] && config["accelerometer"]["offset"]) {
        accel_offset_x_ = config["accelerometer"]["offset"]["x"].as<double>();
        accel_offset_y_ = config["accelerometer"]["offset"]["y"].as<double>();
        accel_offset_z_ = config["accelerometer"]["offset"]["z"].as<double>();
        RCLCPP_INFO(this->get_logger(), "Offsets de aceleración cargados desde YAML");
      }

      // Cargar offsets de giroscopio
      if (config["gyroscope"] && config["gyroscope"]["offset"]) {
        gyro_offset_x_ = config["gyroscope"]["offset"]["x"].as<double>();
        gyro_offset_y_ = config["gyroscope"]["offset"]["y"].as<double>();
        gyro_offset_z_ = config["gyroscope"]["offset"]["z"].as<double>();
        RCLCPP_INFO(this->get_logger(), "Offsets de giroscopio cargados desde YAML");
      }

      RCLCPP_INFO(this->get_logger(), "Calibración cargada exitosamente desde: %s", calib_file.c_str());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Error al cargar calibración: %s", e.what());
      RCLCPP_WARN(this->get_logger(), "Usando offsets por defecto");
    }
  }

  // Exponential low-pass filter
  // filtered = alpha * raw + (1 - alpha) * previous_filtered
  void apply_lowpass_filter(
    double & accel_x, double & accel_y, double & accel_z,
    double & gyro_x, double & gyro_y, double & gyro_z)
  {
    if (!filter_initialized_) {
      // First measurement: initialize filter state
      filtered_accel_x_ = accel_x;
      filtered_accel_y_ = accel_y;
      filtered_accel_z_ = accel_z;
      filtered_gyro_x_ = gyro_x;
      filtered_gyro_y_ = gyro_y;
      filtered_gyro_z_ = gyro_z;
      filter_initialized_ = true;
      return;
    }

    // Apply exponential smoothing
    filtered_accel_x_ = filter_alpha_ * accel_x + (1.0 - filter_alpha_) * filtered_accel_x_;
    filtered_accel_y_ = filter_alpha_ * accel_y + (1.0 - filter_alpha_) * filtered_accel_y_;
    filtered_accel_z_ = filter_alpha_ * accel_z + (1.0 - filter_alpha_) * filtered_accel_z_;

    filtered_gyro_x_ = filter_alpha_ * gyro_x + (1.0 - filter_alpha_) * filtered_gyro_x_;
    filtered_gyro_y_ = filter_alpha_ * gyro_y + (1.0 - filter_alpha_) * filtered_gyro_y_;
    filtered_gyro_z_ = filter_alpha_ * gyro_z + (1.0 - filter_alpha_) * filtered_gyro_z_;

    // Update output with filtered values
    accel_x = filtered_accel_x_;
    accel_y = filtered_accel_y_;
    accel_z = filtered_accel_z_;
    gyro_x = filtered_gyro_x_;
    gyro_y = filtered_gyro_y_;
    gyro_z = filtered_gyro_z_;
  }

  // Complementary filter to compute orientation from accelerometer and gyroscope
  void compute_orientation(
    double accel_x_g, double accel_y_g, double accel_z_g,
    double gyro_x_dps, double gyro_y_dps, double gyro_z_dps,
    double dt)
  {
    // ========================================
    // STEP 1: Calculate angles from accelerometer (tilt from gravity)
    // ========================================
    // Roll: rotation around X-axis
    float accel_roll = atan2(accel_y_g, accel_z_g) * 180.0 / M_PI;
    
    // Pitch: rotation around Y-axis
    float accel_pitch = atan2(-accel_x_g, sqrt(accel_y_g * accel_y_g + accel_z_g * accel_z_g)) * 180.0 / M_PI;
    
    // Note: Yaw cannot be determined from accelerometer alone (needs magnetometer)
    
    // ========================================
    // STEP 2: Integrate gyroscope rates
    // ========================================
    float gyro_roll_delta = gyro_x_dps * dt;
    float gyro_pitch_delta = gyro_y_dps * dt;
    float gyro_yaw_delta = gyro_z_dps * dt;
    
    // ========================================
    // STEP 3: Complementary filter (fuse accel + gyro)
    // ========================================
    if (!orientation_initialized_) {
      // First measurement: initialize from accelerometer
      roll_ = accel_roll;
      pitch_ = accel_pitch;
      yaw_ = 0.0;  // No reference for yaw without magnetometer
      orientation_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Orientation initialized: Roll=%.2f°, Pitch=%.2f°", roll_, pitch_);
    } else {
      // Complementary filter: combine accelerometer (slow, stable) with gyroscope (fast, drifts)
      roll_ = orientation_alpha_ * (roll_ + gyro_roll_delta) + (1.0 - orientation_alpha_) * accel_roll;
      pitch_ = orientation_alpha_ * (pitch_ + gyro_pitch_delta) + (1.0 - orientation_alpha_) * accel_pitch;
      
      // Yaw: only from gyroscope (will drift over time without magnetometer)
      yaw_ += gyro_yaw_delta;
      
      // Normalize yaw to -180 to +180 degrees
      if (yaw_ > 180.0) yaw_ -= 360.0;
      if (yaw_ < -180.0) yaw_ += 360.0;
    }
  }

  // Convert Euler angles (roll, pitch, yaw) to quaternion
  void euler_to_quaternion(double roll_deg, double pitch_deg, double yaw_deg, 
                          double &qx, double &qy, double &qz, double &qw)
  {
    // Convert to radians
    double roll_rad = roll_deg * M_PI / 180.0;
    double pitch_rad = pitch_deg * M_PI / 180.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;

    // Use tf2 for conversion
    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);

    qx = q.x();
    qy = q.y();
    qz = q.z();
    qw = q.w();
  }

  void imu_callback(const kalman_interfaces::msg::ImuData & imu_data_msg)
  {
    // Calculate time delta
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Avoid division by zero and unrealistic dt values
    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.01;  // Default to 100 Hz if timing is unrealistic
    }

    // Crear mensaje Imu estándar de ROS2
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = "imu_link";

    // Convert raw data to double and apply calibration offsets
    double accel_x = static_cast<double>(imu_data_msg.accel_x) - accel_offset_x_;
    double accel_y = static_cast<double>(imu_data_msg.accel_y) - accel_offset_y_;
    double accel_z = static_cast<double>(imu_data_msg.accel_z) - accel_offset_z_;

    double gyro_x = static_cast<double>(imu_data_msg.gyro_x) - gyro_offset_x_;
    double gyro_y = static_cast<double>(imu_data_msg.gyro_y) - gyro_offset_y_;
    double gyro_z = static_cast<double>(imu_data_msg.gyro_z) - gyro_offset_z_;

    // Apply low-pass filter to reduce sensor noise
    apply_lowpass_filter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

    // Scale and convert acceleration (LSB -> g -> m/s²)
    imu_msg.linear_acceleration.x = (accel_x / accel_scale_) * gravity_;
    imu_msg.linear_acceleration.y = (accel_y / accel_scale_) * gravity_;
    imu_msg.linear_acceleration.z = (accel_z / accel_scale_) * gravity_;

    // Scale and convert angular velocity (LSB -> °/s -> rad/s)
    imu_msg.angular_velocity.x = (gyro_x / gyro_scale_) * deg_to_rad_;
    imu_msg.angular_velocity.y = (gyro_y / gyro_scale_) * deg_to_rad_;
    imu_msg.angular_velocity.z = (gyro_z / gyro_scale_) * deg_to_rad_;

    // Compute orientation using complementary filter
    if (publish_orientation_) {
      // Convert to g's and degrees/second for orientation calculation
      double accel_x_g = accel_x / accel_scale_;
      double accel_y_g = accel_y / accel_scale_;
      double accel_z_g = accel_z / accel_scale_;
      double gyro_x_dps = gyro_x / gyro_scale_;
      double gyro_y_dps = gyro_y / gyro_scale_;
      double gyro_z_dps = gyro_z / gyro_scale_;

      compute_orientation(accel_x_g, accel_y_g, accel_z_g,
                         gyro_x_dps, gyro_y_dps, gyro_z_dps, dt);

      // Convert Euler angles to quaternion
      double qx, qy, qz, qw;
      euler_to_quaternion(roll_, pitch_, yaw_, qx, qy, qz, qw);

      imu_msg.orientation.x = qx;
      imu_msg.orientation.y = qy;
      imu_msg.orientation.z = qz;
      imu_msg.orientation.w = qw;

      // Set orientation covariance (diagonal matrix)
      // Lower values = more confident
      imu_msg.orientation_covariance[0] = 0.01;  // roll variance
      imu_msg.orientation_covariance[4] = 0.01;  // pitch variance
      imu_msg.orientation_covariance[8] = 0.1;   // yaw variance (higher, no magnetometer)
    } else {
      // Set orientation covariance to -1 to indicate orientation is not available
      imu_msg.orientation_covariance[0] = -1.0;
    }

    // Set linear acceleration covariance
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

    // Set angular velocity covariance
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[4] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;

    // Publicar mensaje
    imu_pub_->publish(imu_msg);

    RCLCPP_DEBUG(this->get_logger(),
      "IMU - Accel: [%.2f, %.2f, %.2f] Gyro: [%.2f, %.2f, %.2f]",
      imu_data_msg.accel_x, imu_data_msg.accel_y, imu_data_msg.accel_z,
      imu_data_msg.gyro_x, imu_data_msg.gyro_y, imu_data_msg.gyro_z);
  }

  rclcpp::Subscription<kalman_interfaces::msg::ImuData>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Scaling parameters
  double accel_scale_;  // LSB/g
  double gyro_scale_;   // LSB/°/s
  double gravity_;      // m/s²
  double deg_to_rad_;   // conversion constant

  // Calibration offsets (bias)
  double accel_offset_x_;  // LSB
  double accel_offset_y_;  // LSB
  double accel_offset_z_;  // LSB
  double gyro_offset_x_;   // LSB
  double gyro_offset_y_;   // LSB
  double gyro_offset_z_;   // LSB

  // Low-pass filter parameters
  double filter_alpha_;  // Filter coefficient (0.0-1.0)
  bool filter_initialized_;  // Flag to track if filter is initialized

  // Filter state (previous filtered values)
  double filtered_accel_x_, filtered_accel_y_, filtered_accel_z_;
  double filtered_gyro_x_, filtered_gyro_y_, filtered_gyro_z_;

  // Orientation estimation parameters
  double orientation_alpha_;  // Complementary filter coefficient
  bool publish_orientation_;
  bool orientation_initialized_;
  double roll_, pitch_, yaw_;  // Orientation angles in degrees

  // Timing
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
