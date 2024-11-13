
#include "canadarm_panel.hpp"

#include <stdio.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <memory>

#include "cfe_msgs/msg/canadarm_app_cmdt.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace brash_application_tools
{
CanadarmPanel::CanadarmPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * poses_layout = new QVBoxLayout;
  QPushButton* open_button = new QPushButton("Open", this);
  QPushButton* close_button = new QPushButton("Close", this);
  QPushButton* random_button = new QPushButton("Random", this);
     
  poses_layout->addWidget(open_button);
  poses_layout->addWidget(close_button);
  poses_layout->addWidget(random_button);
    
  connect(open_button, SIGNAL(clicked()), this, SLOT(openClicked()), Qt::UniqueConnection);
  connect(close_button, SIGNAL(clicked()), this, SLOT(closeClicked()), Qt::UniqueConnection);
  connect(random_button, SIGNAL(clicked()), this, SLOT(randomClicked()), Qt::UniqueConnection);
      
  setLayout(poses_layout);

  // Create the publisher.
  node_ = std::make_shared<rclcpp::Node>("canadarm_panel");
  publish_state_ = node_->create_publisher<cfe_msgs::msg::CanadarmAppCmdt>("/groundsystem/canadarm_app_cmd", 10);

}

void CanadarmPanel::sendPose( int _mode)
{
   cfe_msgs::msg::CanadarmAppCmdt msg;
   msg.cmd_header.sec.function_code = 1;
   msg.pose_id = _mode;
   publish_state_->publish(msg);
}


void CanadarmPanel::openClicked()
{
   sendPose(0);
}

void CanadarmPanel::closeClicked()
{
   sendPose(1);
}


void CanadarmPanel::randomClicked()
{
   sendPose(2);
}


void CanadarmPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void CanadarmPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
}

}  

PLUGINLIB_EXPORT_CLASS(brash_application_tools::CanadarmPanel, rviz_common::Panel)

