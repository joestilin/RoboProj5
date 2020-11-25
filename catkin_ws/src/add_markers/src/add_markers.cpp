#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



class add_markers_pubsub {
	public:
		add_markers_pubsub() {
			marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			amcl_pose_sub = n.subscribe("amcl_pose", 1, &add_markers_pubsub::amcl_pose_callback, this);			
		}
		
		void place_marker(double x, double y) {
			// acceptable odom position error from pickup and dropoff points
			double tolerance = 0.5;
 
			// if at pickup or dropoff points

			
			// Set our initial shape type to be a cube
		  	uint32_t shape = visualization_msgs::Marker::CUBE;

			visualization_msgs::Marker marker;
			// Set the frame ID and timestamp.  See the TF tutorials for information on these.
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();

			// Set the namespace and id for this marker.  This serves to create a unique ID
			// Any marker sent with the same namespace and id will overwrite the old one
			marker.ns = "basic_shapes";
			marker.id = 0;

			// Set the marker type.
			marker.type = shape;

			// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
			marker.action = visualization_msgs::Marker::ADD;

			// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

		  
			marker.pose.position.x = x;
			marker.pose.position.y = y;

			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;

			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = 1.0;
			marker.scale.y = 1.0;
			marker.scale.z = 1.0;

			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;

			marker.lifetime = ros::Duration();

			marker_pub.publish(marker);
  }

		void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
			// process incoming odom data to determine if robot is at pickup and dropoff points
    		// test subscribe
			//ROS_INFO("%f\n", msg->pose.pose.position.x);
			
			place_marker(msg->pose.pose.position.x, msg->pose.pose.position.y);	
		}

	private:
		ros::NodeHandle n;
		ros::Publisher marker_pub;
		ros::Subscriber amcl_pose_sub;
		double initial_x;
		double initial_y;
	};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");

	add_markers_pubsub add_markers;

	ros::spin();

	return 0;
  
}

