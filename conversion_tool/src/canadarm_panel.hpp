
#ifndef CANADARM_PANEL_HPP_
#define CANADARM_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <memory>

#include "cfe_msgs/msg/canadarm_app_cmdt.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#endif

class QLineEdit;

namespace brash_application_tools
{

class CanadarmPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit CanadarmPanel(QWidget * parent = 0);

  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;


protected Q_SLOTS:

  void openClicked();
  void closeClicked();
  void randomClicked();


protected:

  void sendPose( int _mode);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<cfe_msgs::msg::CanadarmAppCmdt>::SharedPtr publish_state_;

};

}  // end namespace

#endif  // CANADARM_PANEL_HPP_

