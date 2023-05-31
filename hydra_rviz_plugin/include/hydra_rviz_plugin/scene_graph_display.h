#pragma once

#include <hydra_msgs/DsgUpdate.h>
#include <rviz/message_filter_display.h>

namespace hydra {

class SceneGraphDisplay : public rviz::MessageFilterDisplay<hydra_msgs::DsgUpdate> {
  Q_OBJECT
 public:
  SceneGraphDisplay();

  virtual ~SceneGraphDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const hydra_msgs::DsgUpdate::ConstPtr& msg);
};

}  // namespace hydra
