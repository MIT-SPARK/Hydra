#include <kimera_scene_graph/humans_serialization.h>
#include <pluginlib/class_list_macros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace kimera {

class HumansRosbagSerializer : public HumansSerializer {
 public:
  HumansRosbagSerializer() : HumansSerializer() {}
  ~HumansRosbagSerializer() = default;

  virtual auto start(const std::string& name) -> bool override {
    if (not make_directories_(serialization_dir_)) {
      LOG(ERROR) << "Serialization failed";
      return false;
    }

    std::string filepath = serialization_dir_ + "/" + name + ".bag";
    bag_.open(filepath, rosbag::bagmode::Write);
    return true;
  }

  virtual void stop() override { bag_.close(); }

  virtual void write(const std::string& name,
                     const visualization_msgs::Marker& marker) override {
    if (not bag_.isOpen()) {
      LOG(ERROR) << "Bag is not open. Failed to write " << name;
      return;
    }
    if (name.empty()) {
      LOG(ERROR) << "Dropping marker with empty topic";
      return;
    }
    bag_.write(name, ros::Time(1), marker);
  }

  virtual void write(const std::string& name,
                     const graph_cmr_ros::SMPLList& smpl_list) override {
    if (not bag_.isOpen()) {
      LOG(ERROR) << "Bag is not open. Failed to write " << name;
      return;
    }
    if (name.empty()) {
      LOG(ERROR) << "Dropping SMPLList with empty topic";
      return;
    }
    bag_.write(name, ros::Time(1), smpl_list);
  }

  virtual void write(const std::string& name,
                     const ColorPointCloud& cloud) override {
    if (not bag_.isOpen()) {
      LOG(ERROR) << "Bag is not open. Failed to write " << name;
      return;
    }
    if (name.empty()) {
      LOG(ERROR) << "Dropping pointcloud with empty topic";
      return;
    }
    bag_.write(name, ros::Time(1), cloud);
  }

 private:
  rosbag::Bag bag_;
};

class HumansRosbagDeserializer : public HumansDeserializer {
 public:
  HumansRosbagDeserializer() : HumansDeserializer() {}
  ~HumansRosbagDeserializer() = default;

  virtual auto start(const std::string& filepath) -> bool override {
    if (not path_exists_(filepath)) {
      LOG(ERROR) << "Serialization failed";
      return false;
    }

    try {
      bag_.open(filepath, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
      LOG(ERROR) << "Failed to load rosbag at: " << filepath;
      LOG(ERROR) << e.what();
      return false;
    }

    view_.reset(new rosbag::View(bag_));
    return true;
  }

  struct BagIter : public HumansDeserializer::Iter {
    BagIter(const rosbag::View::iterator& bag_iter) : bag_iter_(bag_iter) {}

    virtual auto get() -> EntryInfo override {
      rosbag::MessageInstance m = *bag_iter_;
      if (m.isType<visualization_msgs::Marker>()) {
        return EntryInfo(m.getTopic(),
                         m.instantiate<visualization_msgs::Marker>());
      } else if (m.isType<graph_cmr_ros::SMPLList>()) {
        return EntryInfo(m.getTopic(),
                         m.instantiate<graph_cmr_ros::SMPLList>());
      } else if (m.isType<ColorPointCloud>()) {
        return EntryInfo(m.getTopic(), m.instantiate<ColorPointCloud>());
      } else {
        return EntryInfo();
      }
    }

    virtual void increment() override { ++bag_iter_; }

    virtual auto equals(const Iter::Ptr& other) -> bool override {
      auto other_derived = std::dynamic_pointer_cast<BagIter>(other);
      if (not other_derived) {
        LOG(ERROR) << "Failed to downcast other iterator";
        return false;
      }

      return bag_iter_ == other_derived->bag_iter_;
    }

   private:
    rosbag::View::iterator bag_iter_;
  };

  virtual void stop() override { bag_.close(); }

  virtual auto begin() const -> DeserializerIterator override {
    CHECK(view_);
    return DeserializerIterator(std::make_shared<BagIter>(view_->begin()));
  }

  virtual auto end() const -> DeserializerIterator override {
    CHECK(view_);
    return DeserializerIterator(std::make_shared<BagIter>(view_->end()));
  }

 private:
  rosbag::Bag bag_;
  std::unique_ptr<rosbag::View> view_;
};

}  // namespace kimera

PLUGINLIB_EXPORT_CLASS(kimera::HumansRosbagSerializer, kimera::HumansSerializer)
PLUGINLIB_EXPORT_CLASS(kimera::HumansRosbagDeserializer,
                       kimera::HumansDeserializer)
