

#include "gtest_helpers.h"
#include "inertial_sense_ros.h"

InertialSense IS_;

void connect_rtk_client(const std::string& rtk_correction_protocol, const std::string& rtk_server_IP, const int rtk_server_port)
{
  std::string rtk_server_mount = "";
  std::string rtk_server_username = "";
  std::string rtk_server_password = "";

  int rtk_connection_attempt_limit = 1;
  int rtk_connection_attempt_backoff = 2;

  // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
  std::string RTK_connection =  "TCP:" + rtk_correction_protocol + ":" + rtk_server_IP + ":" + std::to_string(rtk_server_port);
  if (!rtk_server_mount.empty() && !rtk_server_username.empty())
  { // NTRIP options
    RTK_connection += ":" + rtk_server_mount + ":" + rtk_server_username + ":" + rtk_server_password;
  }

  int RTK_connection_attempt_count = 0;
  while (RTK_connection_attempt_count < rtk_connection_attempt_limit)
  {
    ++RTK_connection_attempt_count;

    bool connected = IS_.OpenConnectionToServer(RTK_connection);

    if (connected)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("connection_successful"),"Successfully connected to " << RTK_connection << " RTK server");
      break;
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("failed_to_connect_base"),"Failed to connect to base server at " << RTK_connection);

      if (RTK_connection_attempt_count >= rtk_connection_attempt_limit)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("give_up_failed_attempts"),"Giving up after " << RTK_connection_attempt_count << " failed attempts");
      }
      else
      {
        int sleep_duration = RTK_connection_attempt_count * rtk_connection_attempt_backoff;
        RCLCPP_WARN_STREAM(rclcpp::get_logger("retrying_connection"),"Retrying connection in " << sleep_duration << " seconds");
        //ros::Duration(sleep_duration).sleep();
        rclcpp::Rate r(sleep_duration); r.sleep();
      }
    }
  }
}

TEST(ReconnectionTestSuite, testReconnection)
{
    const std::string rtk_correction_protocol = "RTCM3";
    const std::string& rtk_server_IP = "66.219.246.93";
    const int rtk_server_port = 7777;

    connect_rtk_client(rtk_correction_protocol, rtk_server_IP, rtk_server_port);
}
