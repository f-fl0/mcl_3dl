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

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mcl_3dl
{
class MapLoader
{
public:
  MapLoader()
  {
    pnh_.param<std::string>("frame_id", frame_id_, "map");
    pub_mapcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("mapcloud", 1, true);
  }

  bool init()
  {
    std::string map_file;
    if (pnh_.hasParam("map_file"))
    {
      pnh_.getParam("map_file", map_file);

      pcl::PCLPointCloud2 map_cloud;
      if (map_file.empty() || reader_.read(map_file, map_cloud) < 0)
      {
        ROS_ERROR("Could not load the map file: %s", map_file.c_str());
        return false;
      }
      sensor_msgs::PointCloud2 map_cloud_msg;
      pcl_conversions::moveFromPCL(map_cloud, map_cloud_msg);
      map_cloud_msg.header.stamp = ros::Time::now();
      map_cloud_msg.header.frame_id = frame_id_;
      pub_mapcloud_.publish(map_cloud_msg);
      return true;
    }
    return false;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  pcl::PCDReader reader_;
  ros::Publisher pub_mapcloud_;
  std::string frame_id_;
};

}  // namespace mcl_3dl

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "map_loader");

  mcl_3dl::MapLoader map_loader;
  if (!map_loader.init())
  {
    return 1;
  }
  ros::spin();

  return 0;
}
