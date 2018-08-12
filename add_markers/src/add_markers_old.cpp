#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "marker_shapes";
  marker.id = 0;

  // Set the marker type to CUBE
  marker.type = shape;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  
  // Setup the pose of the marker
  marker.pose.position.x = 3.8;
  marker.pose.position.y = 2.6;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
    
  
  while (ros::ok())
  {
  
    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
       
    // Publish the maker at the pickup zone
    ROS_INFO("publish pickup marker");
    marker.pose.position.x = 3.8;
    marker.pose.position.y = 2.6;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 
    marker_pub.publish(marker);

    // Pause 5 seconds, Hide the marker
    ros::Duration(5.0).sleep();
    ROS_INFO("hide pickup marker");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    // Pause 5 seconds, Publish the marker at drop off zone
    ros::Duration(5.0).sleep();
    ROS_INFO("create drop-off marker");
    marker.pose.position.x = 3.8;
    marker.pose.position.y = -0.5;
    marker.action = visualization_msgs::Marker::ADD;
    // 
    marker_pub.publish(marker);

    return 0;
  }
}
