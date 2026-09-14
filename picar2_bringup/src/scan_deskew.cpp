// Motion-compensate a LaserScan using odometry.
//
// The LD19 takes ~100 ms per revolution and the driver stamps the whole
// scan with the time the revolution finished. At 1 m/s the first beams are
// placed 10 cm from where they were measured, and a 1 rad/s turn smears a
// 4 m wall by 40 cm. AMCL and the costmaps take the scan as instantaneous.
//
// Every beam is moved to where the sensor was when it was measured, using
// the odom -> laser transform, and the result is re-binned on the original
// angle grid at the reference time. Output is still a LaserScan in the
// laser frame, so nothing downstream changes. If TF is not available for a
// scan it passes through untouched: a stale or missing scan is worse than
// a skewed one.
//
// Beam timing (ldlidar_component.cpp): the stamp is the end of the
// revolution, time_increment = scan_time / (bins - 1), and with rot_verse
// CCW the driver reverses the native order, so ROS beam j was measured at
// stamp - j * time_increment. reverse_beam_time covers the CW case.
//
// The driver stamps the scan "now" while the EKF's transform runs ~40 ms
// behind, so the stamp itself is never in the buffer yet. The output is
// referenced to the newest transform there is and the handful of beams
// newer than that are extrapolated at constant velocity (max_lead).
//
// This is the C++ port of scan_deskew.py, same maths bin for bin. The
// Python version cost a quarter of a Pi core, nearly all of it rclpy's
// per-message executor overhead on the 56 Hz /tf feed.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

struct Pose2
{
  double x, y, yaw;
};

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// numpy.interp: piecewise linear, clamped to the end values outside xs.
double interp(double x, const std::vector<double> & xs, const std::vector<double> & ys)
{
  if (x <= xs.front()) {return ys.front();}
  if (x >= xs.back()) {return ys.back();}
  auto it = std::upper_bound(xs.begin(), xs.end(), x);
  size_t i = it - xs.begin();       // xs[i-1] <= x < xs[i]
  double w = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
  return ys[i - 1] + w * (ys[i] - ys[i - 1]);
}

}  // namespace

class ScanDeskew : public rclcpp::Node
{
public:
  ScanDeskew()
  : Node("scan_deskew")
  {
    const std::string input = declare_parameter<std::string>("input", "/lidar_node/scan_raw");
    const std::string output = declare_parameter<std::string>("output", "/lidar_node/scan");
    fixed_ = declare_parameter<std::string>("fixed_frame", "odom");
    // Poses are looked up at this many instants per scan and the beams
    // between them interpolated: 16 lookups at 10 Hz is nothing.
    segments_ = declare_parameter<int>("segments", 16);
    // true: beam j measured at stamp - j*dt (driver rot_verse CCW).
    reverse_ = declare_parameter<bool>("reverse_beam_time", true);
    // Beams newer than the newest transform are extrapolated; beyond this
    // the transform is simply stale (EKF hiccup) - pass through.
    max_lead_ = declare_parameter<double>("max_lead", 0.15);

    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(5.0));
    // Own thread for /tf (the default), so the scan callback never starves it.
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);

    auto qos = rclcpp::QoS(5).best_effort();
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output, qos);
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input, qos, [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {on_scan(std::move(msg));});
    report_timer_ = create_wall_timer(std::chrono::seconds(30), [this]() {report();});
  }

private:
  void report()
  {
    if (n_pass_) {
      RCLCPP_WARN(get_logger(), "%d of %d scans passed through without deskew (TF unavailable)",
                  n_pass_, n_pass_ + n_ok_);
    }
    n_pass_ = n_ok_ = 0;
  }

  Pose2 pose_at(const std::string & frame, const rclcpp::Time & t)
  {
    auto tf = buffer_->lookupTransform(fixed_, frame, t);
    return {tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)};
  }

  void on_scan(sensor_msgs::msg::LaserScan::UniquePtr msg)
  {
    const size_t n = msg->ranges.size();
    const double dt = msg->time_increment;
    if (n < 2 || dt <= 0.0) {
      pub_->publish(std::move(msg));
      return;
    }
    const rclcpp::Time t_end(msg->header.stamp, RCL_ROS_TIME);
    std::string frame = msg->header.frame_id;
    if (!frame.empty() && frame[0] == '/') {frame.erase(0, 1);}

    std::vector<double> offs, px, py, pyaw;
    double lead;
    Pose2 ref;
    try {
      auto latest = buffer_->lookupTransform(fixed_, frame, tf2::TimePointZero);
      rclcpp::Time t_ref(latest.header.stamp, RCL_ROS_TIME);
      if (t_ref > t_end) {t_ref = t_end;}
      lead = (t_end - t_ref).seconds();   // beams newer than t_ref
      if (lead > max_lead_) {
        throw tf2::TransformException("transform too old");
      }
      ref = pose_at(frame, t_ref);
      // Sample poses back through the revolution from t_ref.
      const double span = dt * static_cast<double>(n - 1);
      offs.reserve(segments_ + 2);
      px.reserve(segments_ + 2); py.reserve(segments_ + 2); pyaw.reserve(segments_ + 2);
      for (int k = 0; k <= segments_; ++k) {
        const double o = span * static_cast<double>(k) / static_cast<double>(segments_);
        Pose2 p = pose_at(frame, t_ref - rclcpp::Duration::from_seconds(o));
        offs.push_back(o); px.push_back(p.x); py.push_back(p.y); pyaw.push_back(p.yaw);
      }
    } catch (const tf2::TransformException &) {
      ++n_pass_;
      pub_->publish(std::move(msg));
      return;
    }
    ++n_ok_;

    // Unwrap yaw so interpolation does not jump across +-pi.
    for (size_t k = 1; k < pyaw.size(); ++k) {
      double d = pyaw[k] - pyaw[k - 1];
      d -= 2.0 * M_PI * std::round(d / (2.0 * M_PI));
      pyaw[k] = pyaw[k - 1] + d;
    }
    if (lead > 0.0) {
      // Prepend an extrapolated pose at -lead so the interpolation covers
      // the beams measured after t_ref (constant velocity over <= max_lead).
      const double h = offs[1] - offs[0];
      const double vx = (px[0] - px[1]) / h, vy = (py[0] - py[1]) / h,
        vyaw = (pyaw[0] - pyaw[1]) / h;
      offs.insert(offs.begin(), -lead);
      px.insert(px.begin(), px[0] + vx * lead);
      py.insert(py.begin(), py[0] + vy * lead);
      pyaw.insert(pyaw.begin(), pyaw[0] + vyaw * lead);
    }

    const double c_e = std::cos(-ref.yaw), s_e = std::sin(-ref.yaw);
    std::vector<float> out(n, std::numeric_limits<float>::infinity());
    for (size_t j = 0; j < n; ++j) {
      const float r = msg->ranges[j];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {continue;}
      // Seconds before t_ref this beam was measured (negative = after it).
      const double age = (reverse_ ? static_cast<double>(j) : static_cast<double>(n - 1 - j)) * dt - lead;
      const double sx = interp(age, offs, px), sy = interp(age, offs, py),
        syaw = interp(age, offs, pyaw);
      const double ang = msg->angle_min + static_cast<double>(j) * msg->angle_increment;
      // Beam endpoint in the fixed frame...
      const double wx = sx + r * std::cos(ang + syaw), wy = sy + r * std::sin(ang + syaw);
      // ...seen from the sensor at the reference time.
      const double dx = wx - ref.x, dy = wy - ref.y;
      const double lx = dx * c_e - dy * s_e, ly = dx * s_e + dy * c_e;
      const double nr = std::hypot(lx, ly);
      double na = std::atan2(ly, lx) - msg->angle_min;
      na = std::fmod(na, 2.0 * M_PI);
      if (na < 0.0) {na += 2.0 * M_PI;}
      long b = std::lround(na / msg->angle_increment);
      if (b < 0) {b = 0;}
      if (b >= static_cast<long>(n)) {b = static_cast<long>(n) - 1;}
      // A bin that receives two beams keeps the nearer one, as the driver
      // does when two native beams land in one bin.
      if (nr < out[b]) {out[b] = static_cast<float>(nr);}
    }

    auto res = std::make_unique<sensor_msgs::msg::LaserScan>();
    res->header = msg->header;
    res->header.stamp = (t_end - rclcpp::Duration::from_seconds(lead));   // = t_ref
    res->angle_min = msg->angle_min;
    res->angle_max = msg->angle_max;
    res->angle_increment = msg->angle_increment;
    res->time_increment = 0.0f;            // every beam now refers to stamp
    res->scan_time = msg->scan_time;
    res->range_min = msg->range_min;
    res->range_max = msg->range_max;
    res->ranges = std::move(out);
    pub_->publish(std::move(res));
  }

  std::string fixed_;
  int segments_{16};
  bool reverse_{true};
  double max_lead_{0.15};
  int n_pass_{0}, n_ok_{0};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanDeskew>());
  rclcpp::shutdown();
  return 0;
}
