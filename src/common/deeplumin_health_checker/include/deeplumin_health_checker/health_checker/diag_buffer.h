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

#ifndef deeplumin_HEALTH_CHECKER_HEALTH_CHECKER_DIAG_BUFFER_H
#define deeplumin_HEALTH_CHECKER_HEALTH_CHECKER_DIAG_BUFFER_H
// headers in Autoare
#include <deeplumin_health_checker/constants.h>
#include <deeplumin_msgs/DiagnosticStatusArray.h>

// headers in STL
#include <map>
#include <mutex>
#include <string>
#include <vector>

// headers in ROS
#include <ros/ros.h>

namespace deeplumin_health_checker
{
class DiagBuffer
{
public:
  DiagBuffer(ErrorKey key, ErrorType type, std::string description,
             double buffer_duration);
  void addDiag(deeplumin_msgs::DiagnosticStatus status);
  deeplumin_msgs::DiagnosticStatusArray getAndClearData();
  const ErrorType type;
  const std::string description;

private:
  using AwDiagStatus = deeplumin_msgs::DiagnosticStatus;
  std::mutex mtx_;
  ErrorLevel getErrorLevel();
  void updateBuffer();
  ErrorKey key_;
  ros::Duration buffer_duration_;
  std::map<ErrorLevel, deeplumin_msgs::DiagnosticStatusArray> buffer_;
  deeplumin_msgs::DiagnosticStatusArray filterBuffer(
    ros::Time now, ErrorLevel level);
  ros::Publisher status_pub_;
  bool isOlderTimestamp(const deeplumin_msgs::DiagnosticStatus &a,
                        const deeplumin_msgs::DiagnosticStatus &b);
};
}  // namespace deeplumin_health_checker

#endif  // deeplumin_HEALTH_CHECKER_HEALTH_CHECKER_DIAG_BUFFER_H
