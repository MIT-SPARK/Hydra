#pragma once
#include <glog/logging.h>
#include <graph_cmr_ros/SMPLList.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <boost/filesystem.hpp>

#include <memory>
#include <string>

#include "kimera_scene_graph/common.h"

namespace kimera {

class HumansSerializer {
 public:
  HumansSerializer() = default;
  virtual ~HumansSerializer() = default;

  virtual void initialize(const ros::NodeHandle& nh) {
    nh.param("serialization_dir", serialization_dir_, serialization_dir_);
  }

  virtual auto start(const std::string& name) -> bool = 0;

  virtual void stop() = 0;

  virtual void write(const std::string& name,
                     const visualization_msgs::Marker& marker) = 0;

  virtual void write(const std::string& name,
                     const graph_cmr_ros::SMPLList& smpl_list) = 0;

  virtual void write(const std::string& name, const ColorPointCloud& cloud) = 0;

 protected:
  std::string serialization_dir_{"."};

  inline auto make_directories_(const std::string& filepath) -> bool {
    namespace fs = boost::filesystem;
    fs::path path(filepath);
    if (fs::exists(path)) {
      return true;
    }

    LOG(INFO) << "Serialization directory: " << filepath << " doesn't exist";

    if (not fs::create_directories(path)) {
      LOG(ERROR) << "Failed creating path: " << filepath;
      return false;
    }

    return true;
  }
};

// TODO(nathan) this should really be templated, but...
class EntryInfo {
 public:
  enum class Type { MARKER, SMPL, PCL, UNKNOWN } type;
  std::string topic;

  EntryInfo() : type(Type::UNKNOWN) {}

  EntryInfo(const std::string& name,
            const visualization_msgs::Marker::ConstPtr& marker)
      : type(Type::MARKER), topic(name), marker_handle_(marker) {}

  EntryInfo(const std::string& name,
            const graph_cmr_ros::SMPLList::ConstPtr& list)
      : type(Type::SMPL), topic(name), smpl_handle_(list) {}

  EntryInfo(const std::string& name, const ColorPointCloud::ConstPtr& cloud)
      : type(Type::PCL), topic(name), pcl_handle_(cloud) {}

  inline auto getMarker() const -> visualization_msgs::Marker::ConstPtr {
    if (type != Type::MARKER) {
      return visualization_msgs::Marker::ConstPtr();
    }
    return marker_handle_;
  }

  inline auto getSMPL() const -> graph_cmr_ros::SMPLList::ConstPtr {
    if (type != Type::SMPL) {
      return graph_cmr_ros::SMPLList::ConstPtr();
    }
    return smpl_handle_;
  }

  inline auto getPCL() const -> ColorPointCloud::ConstPtr {
    if (type != Type::PCL) {
      return ColorPointCloud::ConstPtr();
    }
    return pcl_handle_;
  }

 private:
  visualization_msgs::Marker::ConstPtr marker_handle_{nullptr};
  graph_cmr_ros::SMPLList::ConstPtr smpl_handle_{nullptr};
  ColorPointCloud::ConstPtr pcl_handle_{nullptr};
};

// forward declaration to enable polymorphism
class DeserializerIterator;

class HumansDeserializer {
 public:
  HumansDeserializer() = default;
  virtual ~HumansDeserializer() = default;

  virtual void initialize(const ros::NodeHandle&) {}

  virtual auto start(const std::string& filepath) -> bool = 0;

  virtual void stop() = 0;

  virtual auto begin() const -> DeserializerIterator = 0;

  virtual auto end() const -> DeserializerIterator = 0;

  struct Iter {
    typedef std::shared_ptr<Iter> Ptr;
    virtual ~Iter() = default;
    virtual auto get() -> EntryInfo = 0;
    virtual void increment() = 0;
    virtual auto equals(const Iter::Ptr& other) -> bool = 0;
  };

 protected:
  inline auto path_exists_(const std::string& filepath) -> bool {
    namespace fs = boost::filesystem;
    return fs::exists(fs::path(filepath));
  }
};

class DeserializerIterator {
 public:
  explicit DeserializerIterator(const HumansDeserializer::Iter::Ptr& info)
      : iter_info_(info) {}

  inline auto operator*() -> EntryInfo {
    CHECK(iter_info_);
    return iter_info_->get();
  }

  inline auto operator++() -> DeserializerIterator& {
    CHECK(iter_info_);
    iter_info_->increment();
    return *this;
  }

  inline auto operator!=(const DeserializerIterator& other) -> bool {
    CHECK(iter_info_);
    CHECK(other.iter_info_);
    return not(iter_info_->equals(other.iter_info_));
  }

 private:
  HumansDeserializer::Iter::Ptr iter_info_;
};

template <typename SerializationType>
class SerializationHandle {
 public:
  typedef typename SerializationType::Class SerializationBase;
  static constexpr const char* PACKAGE = "kimera_scene_graph";

  SerializationHandle(const ros::NodeHandle& nh,
                      const std::string& filename,
                      const std::string& type = SerializationType::DEFAULT)
      : type_(type) {
    using pluginlib::ClassLoader;
    loader_.reset(
        new ClassLoader<SerializationBase>(PACKAGE, SerializationType::NAME));
    handle_.reset(loader_->createUnmanagedInstance(type_));
    handle_->initialize(nh);

    if (not handle_->start(filename)) {
      LOG(ERROR) << "Starting the serialization failed.";
      cleanup_(false);
    }
  }

  ~SerializationHandle() { cleanup_(); }

  auto get() -> SerializationBase* { return handle_.get(); }

 private:
  std::unique_ptr<pluginlib::ClassLoader<SerializationBase>> loader_{nullptr};
  std::unique_ptr<SerializationBase> handle_{nullptr};
  std::string type_;

  void cleanup_(bool need_stop = true) {
    const bool had_instance = handle_ != nullptr;

    // perform cleanup if needed
    if (had_instance && need_stop) {
      handle_->stop();
    }
    handle_.reset();

    // handle implications of unmanaged plugin instance
    if (loader_ && had_instance) {
      loader_->unloadLibraryForClass(type_);
    }
    loader_.reset();
  }
};

struct SerializerHandleInfo {
  typedef HumansSerializer Class;
  static constexpr const char* NAME = "kimera::HumansSerializer";
  static constexpr const char* DEFAULT = "kimera::HumansRosbagSerializer";
};

struct DeserializerHandleInfo {
  typedef HumansDeserializer Class;
  static constexpr const char* NAME = "kimera::HumansDeserializer";
  static constexpr const char* DEFAULT = "kimera::HumansRosbagDeserializer";
};

}  // namespace kimera
