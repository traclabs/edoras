
#include "mars_rover_panel.hpp"

#include <stdio.h>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QPixmap>
#include <QLabel>
#include <filesystem>
#include <QTimer>

#include <memory>

#include "cfdp_msgs/srv/cfdp_xfr_cmd.hpp"
#include "pluginlib/class_list_macros.hpp"

using namespace std::chrono_literals;

namespace brash_application_tools
{
MarsRoverPanel::MarsRoverPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  QVBoxLayout * cmd_layout = new QVBoxLayout;
  QPushButton* get_image_button = new QPushButton("Get from RosFsw", this);
  
  image_label_ = new QLabel("Image...");
  filestore_edit_ = new QLineEdit(QString("/code/brash/cfdp/rosgsw"));
  image_src_edit_ = new QLineEdit(QString("rover_image.png"));
  image_dst_edit_ = new QLineEdit(QString("transferred_1.png"));
         
  cmd_layout->addWidget(get_image_button);
  cmd_layout->addWidget(image_label_);
  cmd_layout->addWidget(filestore_edit_);
  cmd_layout->addWidget(image_src_edit_);
  cmd_layout->addWidget(image_dst_edit_);

  setLayout(cmd_layout);

  dt_ = 20.0;
  show_image_ = false;
  QTimer* update_timer_ = new QTimer(this);
      
  connect(get_image_button, SIGNAL(clicked()), this, SLOT(getImageFromRosfsw()), Qt::UniqueConnection);
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(timerCb()));
  
  // Start timer
  update_timer_->start(1000); // 1 seconds (argument is in milliseconds)
      
  // Create the timer, start it stopped  
  node_ = std::make_shared<rclcpp::Node>("mars_rover");
    
  client_ = node_->create_client<cfdp_msgs::srv::CfdpXfrCmd>("/cfdp/cmd/get");

}

void MarsRoverPanel::timerCb(){

  if(show_image_)
  {    
    double elapsed = (node_->get_clock()->now() - show_image_start_time_).seconds();
    if( elapsed > dt_)
    {
      RCLCPP_INFO(node_->get_logger(), "Elapsed enough time checking, stop searching for image");
      show_image_ = false;
    } else
      showImage();
  } // if show_image
  
}

void MarsRoverPanel::showImage()
{

   // Check for image   
   filestore_ = (filestore_edit_->text()).toStdString();
   image_ = (image_dst_edit_->text()).toStdString();
   
   std::string filename = filestore_ + "/" + image_; 

  RCLCPP_INFO_STREAM(node_->get_logger(), "Checking if image "<< filename << " is on the ground"); 

   if(std::filesystem::exists(filename))
   {
   RCLCPP_INFO_STREAM(node_->get_logger(), "Image found." << filename << ". Setting show_image=false and showing");
      QImage img(QString(filename.c_str()));
      img = img.scaled(QSize(200, 200));
      
      QPixmap pm = QPixmap::fromImage(img);
      image_label_->setPixmap(pm);
      // Set show_image back to false, as we are showing the image now
      show_image_ = false;
   }
}


void MarsRoverPanel::getImageFromRosfsw()
{
    RCLCPP_INFO(node_->get_logger(), "Sending request for image from rosfsw");
   // ros2 service call /cfdp/cmd/get cfdp_msgs/srv/CfdpXfrCmd "{src: 'rover_image.png', dst: 'rover_image.png', dstid: 2, ack: True}"
    auto request = std::make_shared<cfdp_msgs::srv::CfdpXfrCmd::Request>();
    
    request->src = (image_src_edit_->text()).toStdString();
    request->dst = (image_dst_edit_->text()).toStdString();
    request->dstid = 2; // flight
    request->ack = true;


  client_->async_send_request(request);

  // Start checking for image till dt
  show_image_start_time_ = node_->get_clock()->now();
  show_image_ = true;  
  image_label_->clear();
}


void MarsRoverPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void MarsRoverPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
}

}  

PLUGINLIB_EXPORT_CLASS(brash_application_tools::MarsRoverPanel, rviz_common::Panel)

