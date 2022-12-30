
#include <gtest/gtest.h>
#include "inertial_sense_ros.h"

InertialSense IS_;

void connect_rtk_client(const std::string& RTK_correction_protocol, const std::string& RTK_server_IP, const int RTK_server_port)
{
  std::string RTK_server_mount = "";
  std::string RTK_server_username = "";
  std::string RTK_server_password = "";

  int RTK_connection_attempt_limit = 10;
  int RTK_connection_attempt_backoff = 2;

  // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
  std::string RTK_connection =  "TCP:" + RTK_correction_protocol + ":" + RTK_server_IP + ":" + std::to_string(RTK_server_port);
  if (!RTK_server_mount.empty() && !RTK_server_username.empty())
  { // NTRIP options
    RTK_connection += ":" + RTK_server_mount + ":" + RTK_server_username + ":" + RTK_server_password;
  }

  int RTK_connection_attempt_count = 0;
  while (RTK_connection_attempt_count < RTK_connection_attempt_limit)
  {
    ++RTK_connection_attempt_count;

    bool connected = IS_.OpenConnectionToServer(RTK_connection);

    if (connected)
    {
      ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
      break;
    }
    else
    {
      ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

      if (RTK_connection_attempt_count >= RTK_connection_attempt_limit)
      {
        ROS_ERROR_STREAM("Giving up after " << RTK_connection_attempt_count << " failed attempts");
      }
      else
      {
        int sleep_duration = RTK_connection_attempt_count * RTK_connection_attempt_backoff;
        ROS_WARN_STREAM("Retrying connection in " << sleep_duration << " seconds");
        ros::Duration(sleep_duration).sleep();
      }
    }
  }
}

TEST(ReconnectionTestSuite, testReconnection)
{
    const std::string RTK_correction_protocol = "RTCM3";
    const std::string& RTK_server_IP = "127.0.0.1";
    const int RTK_server_port = 7777;

    connect_rtk_client(RTK_correction_protocol, RTK_server_IP, RTK_server_port);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_publisher");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
