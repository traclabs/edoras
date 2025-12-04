/**
 * @file arm_ground_command.cpp
 */
#include <edoras_demos/arm_ground_command.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @function ArmGroundCommand
 * @brief Constructor
 */
ArmGroundCommand::ArmGroundCommand(rclcpp::Node::SharedPtr _nh) :
  nh_(_nh)
{
}

/**
 * @function stop
 */
void ArmGroundCommand::stop()
{
  server_.reset();
}

/**
 * @function init
 */
void ArmGroundCommand::init(std::string _reference_frame, std::string _output_topic)
{
  //rclcpp::Duration(0.1).sleep();
  RCLCPP_INFO(nh_->get_logger(), "Initializing IM server...");
  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("arm_ground_command",
									   nh_->get_node_base_interface(),
									   nh_->get_node_clock_interface(),
									   nh_->get_node_logging_interface(),
									   nh_->get_node_topics_interface(),
									   nh_->get_node_services_interface());

  handle_cmd_ = menu_handler_.insert( "Send command pose", std::bind(&ArmGroundCommand::processFeedback, this, _1));
  //handle_coll_ = menu_handler_.insert( "Check Collision Status", std::bind(&ArmGroundCommand::processFeedback, this, _1));

  tf2::Vector3 position;
  reference_frame_ = _reference_frame;
  
  position = tf2::Vector3( 0, 0, 0);
  make6DofMarker( false, visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D,
		  position, true, reference_frame_ );

  pose_topic_ = _output_topic;
  pose_pub_ = nh_->create_publisher<geometry_msgs::msg::Pose>(pose_topic_, 10);
  
  server_->applyChanges();
}

/**
 * @function makeBox
 */
visualization_msgs::msg::Marker ArmGroundCommand::makeBox( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = 0.10; //msg.scale * 0.4;
  marker.scale.y = 0.10; //msg.scale * 0.4;
  marker.scale.z = 0.10; //msg.scale * 0.4;
  marker.color.r = 0.8;
  marker.color.g = 0.1;
  marker.color.b = 0.8;
  marker.color.a = 0.5;

  return marker;
}


/**
 * @function makeBoxControl
 */
visualization_msgs::msg::InteractiveMarkerControl& ArmGroundCommand::makeBoxControl( visualization_msgs::msg::InteractiveMarker &msg )
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( this->makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

/**
 * @function processFeedback
 */
void ArmGroundCommand::processFeedback( const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback )
{
  std::ostringstream s;
  s << "* Feedback from marker '" << feedback->marker_name;

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
    {

      //RCLCPP_INFO_STREAM( nh_->get_logger(), s.str() << ": menu item " << feedback->menu_entry_id << " clicked");
      if( feedback->menu_entry_id == handle_cmd_ )
      {
        RCLCPP_INFO( nh_->get_logger(), "Send command pose, frame id: %s", goal_pose_.header.frame_id.c_str());
        pose_pub_->publish(goal_pose_.pose);
      }

    }
    break;

    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      goal_pose_.pose = feedback->pose;
      goal_pose_.header = feedback->header;
    }
    break;

  } // switch

  server_->applyChanges();
}

/**
 * @function make6DofMarkers
 */
void ArmGroundCommand::make6DofMarker( bool fixed, unsigned int interaction_mode,
				  const tf2::Vector3& position, bool show_6dof,
				  std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1.0;

  int_marker.name = "goal";
  int_marker.description = "6dof goal";

  // insert a box
  makeBoxControl(int_marker);
  int_marker.controls[0].interaction_mode = interaction_mode;

  visualization_msgs::msg::InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = "command_pose";
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&ArmGroundCommand::processFeedback, this, _1));
  if (interaction_mode != visualization_msgs::msg::InteractiveMarkerControl::NONE)
    menu_handler_.apply( *server_, int_marker.name );
}


/**
 * @function makeMenuMarker
 */
void ArmGroundCommand::makeMenuMarker( const tf2::Vector3& position,
			               std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1;

  int_marker.name = "context_menu";
  int_marker.description = "Context Menu\n(Right Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  control.name = "menu_only_control";

  visualization_msgs::msg::Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&ArmGroundCommand::processFeedback,this,_1));
  menu_handler_.apply( *server_, int_marker.name );
}

/**
 * @function makeButtonMarker
 */
void ArmGroundCommand::makeButtonMarker( const tf2::Vector3& position,
				         std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1;

  int_marker.name = "button";
  int_marker.description = "Button\n(Left Click)";

  visualization_msgs::msg::InteractiveMarkerControl control;

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
  control.name = "button_control";

  visualization_msgs::msg::Marker marker = makeBox( int_marker );
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&ArmGroundCommand::processFeedback, this, _1));
}

/**
 * @function makeMovingMarker
 */
void ArmGroundCommand::makeMovingMarker( const tf2::Vector3& position,
				    std::string frame_id)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.pose.position.x = position.getX();
  int_marker.pose.position.y = position.getY();
  int_marker.pose.position.z = position.getZ();  
  int_marker.scale = 1;

  int_marker.name = "moving";
  int_marker.description = "Marker Attached to a\nMoving Frame";

  visualization_msgs::msg::InteractiveMarkerControl control;

  tf2::Quaternion orien(0, 0, 0, 1);
  control.orientation = tf2::toMsg(orien);
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeBox(int_marker) );
  int_marker.controls.push_back(control);

  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&ArmGroundCommand::processFeedback, this, _1));
}

  
void ArmGroundCommand::saveMarker( visualization_msgs::msg::InteractiveMarker int_marker )
{
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, std::bind(&ArmGroundCommand::processFeedback, this, _1));
}

/////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("arm_ground_command");

  node->declare_parameter("reference_frame", std::string("ground/big_arm_link_1"));
  node->declare_parameter("output_topic", std::string("command_pose"));  

  std::string reference_frame;
  std::string output_topic;
  node->get_parameter("reference_frame", reference_frame);
  node->get_parameter("output_topic", output_topic);  
  
  ArmGroundCommand agc(node);
  agc.init(reference_frame, output_topic);
  
  rclcpp::spin(node);
  agc.stop();
  rclcpp::shutdown();
}

