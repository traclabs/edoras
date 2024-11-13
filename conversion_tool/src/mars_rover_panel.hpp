
#ifndef MARS_ROVER_PANEL_HPP_
#define MARS_ROVER_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <memory>

#include "cfdp_msgs/srv/cfdp_xfr_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#endif

class QLineEdit;
class QLabel;

namespace brash_application_tools
{

class MarsRoverPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MarsRoverPanel(QWidget * parent = 0);

  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;


protected Q_SLOTS:

  void getImageFromRosfsw();
  void timerCb();

protected:
  void showImage();
  
  std::string filestore_;
  std::string image_;
  double dt_;
  bool show_image_;
  rclcpp::Time show_image_start_time_;
  
  QLineEdit* filestore_edit_;
  QLineEdit* image_src_edit_;
  QLineEdit* image_dst_edit_;  
  QLabel* image_label_;


  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<cfdp_msgs::srv::CfdpXfrCmd>::SharedPtr client_;
  
};

}  // end namespace

#endif  // CANADARM_PANEL_HPP_

