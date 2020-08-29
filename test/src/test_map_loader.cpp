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

#include <random>
#include <stdio.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <gtest/gtest.h>

#include <mcl_3dl/map_loader.h>

namespace
{
static const char* MAP_FILE_PARAMETER = "/test_map_loader/map_file";
static const char* TEST_MAP_FILE = "test_map.pcd";
}
TEST(MapLoader, NoParameter)
{
  mcl_3dl::MapLoader map_loader;
  ASSERT_FALSE(map_loader.init());
}
TEST(MapLoader, CannotLoadFile)
{
  ros::NodeHandle nh;
  nh.setParam(MAP_FILE_PARAMETER, "does_not_exit.pcd");

  mcl_3dl::MapLoader map_loader;
  ASSERT_FALSE(map_loader.init());
}
TEST(MapLoader, MapPublished)
{
  std::mt19937 gen(42);
  std::uniform_real_distribution<> xyz_dis(-5.0, 5.0);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 300;
  cloud.height = 1;
  cloud.is_dense = false;
  for (int i = 0; i < cloud.width; i++)
  {
    cloud.points.emplace_back(xyz_dis(gen), xyz_dis(gen), xyz_dis(gen));
  }
  pcl::io::savePCDFileASCII(TEST_MAP_FILE, cloud);

  ros::NodeHandle nh;
  nh.setParam(MAP_FILE_PARAMETER, TEST_MAP_FILE);
  mcl_3dl::MapLoader map_loader;
  ASSERT_TRUE(map_loader.init());
  remove(TEST_MAP_FILE);

  sensor_msgs::PointCloud2::ConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/mapcloud", nh);
  ASSERT_EQ(msg->width, cloud.width);
  ASSERT_EQ(msg->height, cloud.height);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_map_loader");

  return RUN_ALL_TESTS();
}
