/*
 * Copyright 2019 deeplumin Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Masaya Kataoka
 */

#ifndef deeplumin_HEALTH_CHECKER_HEALTH_ANALYZER_HEALTH_ANALYZER_H
#define deeplumin_HEALTH_CHECKER_HEALTH_ANALYZER_HEALTH_ANALYZER_H
// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

// headers in STL
#include <map>
#include <string>
#include <vector>

// headers in deeplumin
#include <deeplumin_health_checker/constants.h>
#include <deeplumin_msgs/SystemStatus.h>

// headers in boost
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

struct topic_property
{
  std::string node_pub;
  std::string node_sub;
};

struct node_property
{
  std::string node_name;
};


class HealthAnalyzer
{
public:
  HealthAnalyzer(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~HealthAnalyzer();

private:
  using graph_t = boost::adjacency_list<boost::listS, boost::vecS,
  boost::bidirectionalS, node_property, topic_property>;
  using vertex_t = graph_t::vertex_descriptor;
  using edge_t = graph_t::edge_descriptor;
  using adjacency_iterator_t = boost::graph_traits<graph_t>::adjacency_iterator;
  using out_edge_iterator_t = boost::graph_traits<graph_t>::out_edge_iterator;

  using AwDiagStatus = deeplumin_msgs::DiagnosticStatus;
  ros::Subscriber system_status_sub_;
  ros::Subscriber topic_statistics_sub_;
  ros::Publisher system_status_summary_pub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  void systemStatusCallback(
    const deeplumin_msgs::SystemStatus::ConstPtr& msg);
  void statisticsCallback(
    const rosgraph_msgs::TopicStatistics::ConstPtr& msg);
  void generateDependGraph(const deeplumin_msgs::SystemStatus& status);
  void addDepend(const rosgraph_msgs::TopicStatistics& statistics);
  deeplumin_msgs::SystemStatus filterSystemStatus(
    const deeplumin_msgs::SystemStatus& status);
    std::vector<std::string> findWarningNodes(
      const deeplumin_msgs::SystemStatus& status);
    std::vector<std::string> findErrorNodes(
      const deeplumin_msgs::SystemStatus& status);
  std::vector<std::string> findRootNodes(
    const std::vector<std::string>& target_nodes);
  boost::optional<vertex_t> getTargetNode(const std::string& target_node);
  int countWarn(const deeplumin_msgs::SystemStatus& msg);
  void writeDot();
  graph_t depend_graph_;
  std::vector<rosgraph_msgs::TopicStatistics> topicStatistics_;
  int warn_nodes_count_threshold_;
  template <typename T> bool
    isAlreadyExist(const std::vector<T>& vector, const T& target) const
  {
    auto result = std::find(vector.begin(), vector.end(), target);
    return (result != vector.end());
  }
};

#endif  // deeplumin_HEALTH_CHECKER_HEALTH_ANALYZER_HEALTH_ANALYZER_H
