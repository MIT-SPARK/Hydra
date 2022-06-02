/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_utils/ros_parser.h"

#include <iostream>

namespace config_parser {

RosParserImpl::RosParserImpl(const ros::NodeHandle& nh, const std::string& name)
    : nh_(nh), name_(name) {}

RosParserImpl::RosParserImpl(const ros::NodeHandle& nh) : RosParserImpl(nh, "") {}

RosParserImpl::RosParserImpl() : RosParserImpl(ros::NodeHandle(), "") {}

RosParserImpl RosParserImpl::child(const std::string& new_name) const {
  // push name onto nodehandle namespace if name isn't empty
  ros::NodeHandle new_nh = (name_ == "") ? nh_ : ros::NodeHandle(nh_, name_);
  return RosParserImpl(new_nh, new_name);
}

std::vector<std::string> RosParserImpl::children() const {
  const std::string resolved_name = nh_.resolveName(name_);
  if (resolved_name == "") {
    return {};
  }

  XmlRpc::XmlRpcValue value;
  nh_.getParam(name_, value);
  if (value.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct) {
    return {};
  }

  std::vector<std::string> children;
  for (const auto& nv_pair : value) {
    children.push_back(nv_pair.first);
  }

  return children;
}

}  // namespace config_parser
