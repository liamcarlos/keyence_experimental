//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/header.hpp> 

#include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>

#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_tcp_client.h>
#include <keyence/impl/messages/high_speed_single_profile.h>
// #include <keyence/impl/messages/change_program.h>
#include <keyence/impl/messages/get_setting.h>
#include <keyence/impl/settings_defs.h>

#include <boost/bind/bind.hpp>

#include "boost/ref.hpp"
#include "boost/make_shared.hpp"

// #include "keyence_interfaces/srv/change_program.hpp"

// keyence protocol / profile related defines
static const std::string KEYENCE_DEFAULT_TCP_PORT = "24691";
static const std::string KEYENCE_DEFAULT_TCP_PORT_HS = "24692";

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

// default values for parameters
const static std::string DEFAULT_FRAME_ID = "lj_v7080_optical_frame";

// local types
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

// Prototype for function that converts a given profile to
// a PCL point cloud
int unpackProfileToPointCloud(const keyence::ProfileInformation& info,
                              const std::vector<int>& points,
                              pcl::PointCloud<pcl::PointXYZ>& msg,
                              bool cnv_inf_pts,
                              std::shared_ptr<rclcpp::Node> node);

/**
 * @brief Given a @e client, makes a request to figure out if the given @e program
 * is continuously triggered or not.
 */
bool isProgramContinuouslyTriggered(keyence::TcpClient& client, uint8_t program)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(program);
  uint8_t category = keyence::setting::program::TriggerMode::category;
  uint8_t item = keyence::setting::program::TriggerMode::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  if (!resp.good())
  {
    throw keyence::KeyenceException("Controller responded but errored on request");
  }

  uint8_t result_id = resp.body.data[0];

  if (result_id == keyence::setting::program::TriggerMode::continuous_trigger)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Get's the active program number by making a settings-read request and pulling
 * the number out of the response header.
 */
uint8_t getActiveProgramNumber(keyence::TcpClient& client)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(0);
  uint8_t category = keyence::setting::program::TriggerMode::category;
  uint8_t item = keyence::setting::program::TriggerMode::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  return resp.header.active_program_no;
}

/**
 * @brief Get's the commanded sampling frequency of the auto-trigger setting
 * for the given program. Ranges from 10 Hz - 64 kHz. This driver can currently
 * do only about 200 Hz. The sensor can only do about 1 kHz over TCP/IP with
 * its high speed mode.
 */
double getProgramSamplingRate(keyence::TcpClient& client, uint8_t program)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(program);
  uint8_t category = keyence::setting::program::SamplingPeriod::category;
  uint8_t item = keyence::setting::program::SamplingPeriod::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  if (!resp.good())
  {
    throw keyence::KeyenceException("Controller responded but errored on request");
  }

  uint8_t result_id = resp.body.data[0];

  switch (result_id)
  {
  case keyence::setting::program::SamplingPeriod::freq_10hz:
    return 10.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_20hz:
    return 20.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_50hz:
    return 50.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_100hz:
    return 100.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_200hz:
    return 200.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_500hz:
    return 500.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_1000hz:
    return 1000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_2000hz:
    return 2000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_4000hz:
    return 4000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_4130hz:
    return 4130.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_8000hz:
    return 8000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_16000hz:
    return 16000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_32000hz:
    return 32000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_64000hz:
    return 64000.0;
    break;
  default:
    throw keyence::KeyenceException("Received unrecognized frequency code: " + std::to_string(result_id));
  }
}

/**
 * @brief Services external ROS requests to change the active program. Will reset
 * activity flag and sampling rate according to the settings of the new program.
 */
// bool changeProgramCallback(keyence_interfaces::srv::ChangeProgram::Request& req,
//                            keyence_interfaces::srv::ChangeProgram::Response& res,
//                            keyence::TcpClient& client, bool& active_flag,
//                            rclcpp::Rate& rate)
// {
//   // ... (implementation removed) ...
// }


int main(int argc, char** argv)
{
  //ros::init(argc, argv, "keyence_lj_driver");
  //ros::NodeHandle nh, pnh("~");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyence_lj_driver");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // ros parameters
  std::string sensor_host;
  std::string frame_id;
  std::string sensor_port;
  double sample_rate;
  node->declare_parameter<std::string>("controller_ip", "192.168.1.153");

  // check required parameters
  if (!node->has_parameter("controller_ip"))
  {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Parameter 'controller_ip' missing. Cannot continue.");
    return -1;
  }

  node->get_parameter("controller_ip", sensor_host);
  node->get_parameter_or("controller_port", sensor_port, KEYENCE_DEFAULT_TCP_PORT);
  node->get_parameter_or("frame_id", frame_id, DEFAULT_FRAME_ID);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempting to connect to %s (TCP %s); expecting a single head attached to port A.",
           sensor_host.c_str(), sensor_port.c_str());

  // setup point cloud message (we reuse single one)
  // TODO: this won't work with nodelets
  Cloud::Ptr pc_msg(new Cloud);
  pc_msg->header.frame_id = frame_id;
  pc_msg->is_dense = false; // cloud could have NaNs
  // message is essentially a line-strip of points
  pc_msg->height = 1;

  // set up profile cloud publisher
  //ros::Publisher pub = nh.advertise<Cloud>("profiles", 100);
  //auto pub = node->create_publisher<Cloud>("profiles", 100);
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("profiles", 100);
  //auto pub = node->create_publisher<pcl::PointCloud<pcl::PointXYZ>>("profiles", 100);

  bool active_flag = true;

  while (rclcpp::ok())
  {
    try
    {
      // Establish communications
      keyence::TcpClient keyence(sensor_host, sensor_port);

      auto active_program = getActiveProgramNumber(keyence);
      active_flag = isProgramContinuouslyTriggered(keyence, active_program);
      sample_rate = getProgramSamplingRate(keyence, active_program);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connection established to Keyence controller.");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Beginning with program %hhu with a sampling rate of %f",
               active_program, sample_rate);
      if (!active_flag)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Note that program %hhu is not configured for continuous sampling",
                 active_program);
      }

      rclcpp::Rate sleeper (sample_rate);

      // rclcpp::ServiceBase program_server =
      //     node.advertiseService<keyence::command::ChangeProgram::Request,
      //                         keyence::command::ChangeProgram::Response>(
      //         "change_program", boost::bind(changeProgramCallback, boost::placeholders::_1, boost::placeholders::_2, boost::ref(keyence),
      //                                       boost::ref(active_flag), boost::ref(sleeper)));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Keyence connection established");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Attempting to publish at %.2f Hz.", sample_rate);

      // Main loop
      sleeper.reset();
      while (rclcpp::ok())
      {
        sleeper.sleep();
        //executor.spin_once();
        executor.spin_some();

        // avoid interacting with sensor if there are no publishers
        // TODO: maybe we should actually always poll sensor, but just not
        //       publish anything (although unpacking + publishing is cheap)
        if (pub->get_subscription_count() == 0)
        {
          // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No (more) subscribers. Not polling sensor.");
          continue;
        }

        if (!active_flag)
        {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensor is disabled via the 'activity flag'.");
          continue;
        }

        // unpack profile data
        keyence::command::SingleProfile::Request req;
        auto resp = keyence.sendReceive(req);

        // Check for success
        if (!resp.good())
        {
          PCL_DEBUG_STREAM("Error code in header:\n" << resp.header);
        }
        else
        {
          // convert to pointcloud
          pc_msg->points.clear();
          unpackProfileToPointCloud(resp.body.profile_info, resp.body.profile_points, *pc_msg, true, node);

          sensor_msgs::msg::PointCloud2 ros_cloud;
          pcl::toROSMsg(*pc_msg, ros_cloud);
          // publish pointcloud
          // pub->publish(*pc_msg);
          pub->publish(ros_cloud);
        }
      } // end main loop
    }
    catch (const keyence::KeyenceException& ex)
    {
      PCL_ERROR_STREAM("Keyence Exception Caught:\n" << ex.what());
      PCL_ERROR_STREAM("Attempting reconnection after pause");
      rclcpp::Rate(1.0).sleep();
    }
  }

  return 0;
}

int unpackProfileToPointCloud(const keyence::ProfileInformation& info,
                              const std::vector<int>& points,
                              pcl::PointCloud<pcl::PointXYZ>& msg,
                              bool cnv_inf_pts,
                              std::shared_ptr<rclcpp::Node> node)

{
  // TODO: get proper timestamp from somewhere
  // pcl header stamps are in microseconds
  msg.header.stamp = node->now().nanoseconds() / 1e3;
  msg.width = info.num_profiles;
  cnv_inf_pts = true;

  double x = 0., y = 0., z = 0.;

  msg.points.reserve(info.num_profiles);

  // add points
  for (int i = 0; i < static_cast<int>(points.size()); ++i)
  {
    // convert profile points to SI units (meters)
    x = keyence::unitsToMeters(info.x_start + i * info.x_increment);
    y = 0.0;
    z = keyence::unitsToMeters(static_cast<int>(info.data_unit) * points[i]);

    // filter out 'infinite distance' points
    // REP-117: http://www.ros.org/reps/rep-0117.html
    //  "out of range detections will be represented by +Inf."
    if (points[i] == KEYENCE_INVALID_DATA_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI;
    }

    // device returns two different values that are supposed to be interpreted
    // as out-of-range or 'infinite'. This is the second
    if (points[i] == KEYENCE_DEAD_ZONE_DATA_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI2;
    }

    msg.points.push_back(pcl::PointXYZ(x, y, z));
  }

  return 0;
}
