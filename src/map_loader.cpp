/*
 * Copyright (c) 2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mcl_3dl/map_loader.h>

namespace mcl_3dl
{
class MapLoader : public rclcpp::Node
{
public:
  MapLoader()
    : Node("map_loader")
  {
    this->get_parameter_or("frame_id", frame_id_, std::string("map"));
    pub_mapcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "mapcloud",
        // Transient local is similar to latching in ROS 1.
        rclcpp::QoS(1).transient_local());
  }

  bool init()
  {
    std::string map_file;
    if (!this->get_parameter("map_file", map_file))
    {
      pcl::PCLPointCloud2 map_cloud;
      if (map_file.empty() || reader_.read(map_file, map_cloud) < 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Could not load the map file: %s", map_file.c_str());
        return false;
      }
      // TODO(f-fl0) how to do that without pcl_conversion?
      // sensor_msgs::msg::PointCloud2 map_cloud_msg;
      // pcl_conversions::moveFromPCL(map_cloud, map_cloud_msg);
      // map_cloud_msg.header.stamp = ros::Time::now();
      // map_cloud_msg.header.frame_id = frame_id_;
      // pub_mapcloud_.publish(map_cloud_msg);
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "No map_file parameter.");
    }

    return false;
  }

private:
  pcl::PCDReader reader_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_mapcloud_;
  std::string frame_id_;
};

}  // namespace mcl_3dl

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto map_loader = std::make_shared<mcl_3dl::MapLoader>();
  if (!map_loader->init())
    return 1;
  rclcpp::spin(map_loader);
  rclcpp::shutdown();
  return 0;
}
}  // namespace mcl_3dl
