#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <string>


class add_markers_pubsub {
	public:
		add_markers_pubsub() {
			marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			marker_sub = n.subscribe("marker_position", 1, &add_markers_pubsub::marker_position_callback, this);
			amcl_pose_sub = n.subscribe("amcl_pose", 1, &add_markers_pubsub::amcl_pose_callback, this);
		}
		
		void place_marker(double x, double y, std::string action) {

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
			if (action == "add") {
				marker.action = visualization_msgs::Marker::ADD;
			}
			else if (action == "delete") {
				marker.action = visualization_msgs::Marker::DELETE;
			}

			// Set the pose of the marker.
		  
			marker.pose.position.x = x;
			marker.pose.position.y = y;

			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;

			// Set the scale of the marker
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;

			// Set the color
			marker.color.r = 0.5f;
			marker.color.g = 0.0f;
			marker.color.b = 1.0f;
			marker.color.a = 1.0;

			marker.lifetime = ros::Duration();

			marker_pub.publish(marker);
  }

		void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

			double amcl_x = (double)msg->pose.pose.position.x;
			double amcl_y = (double)msg->pose.pose.position.y;	

			// tolerance, in meters, for amcl pose within pickup / dropoff zones
			double tol = 0.2;	

			if (marker_count == 1) {
				if (sqrt(pow(amcl_x - marker_x, 2) + pow(amcl_y - marker_y, 2)) > tol) {
					place_marker(marker_x, marker_y, "place");
				}
				else {
					place_marker(marker_x, marker_y, "delete");
				}
			}
      
			if (marker_count == 2) {
				if (sqrt(pow(amcl_x - marker_x, 2) + pow(amcl_y - marker_y, 2)) < tol) {
					place_marker(marker_x, marker_y, "place");
				}
				else {
					place_marker(marker_x, marker_y, "delete");
				}
			}
		}
		
		// callback function 
		void marker_position_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
			ROS_INFO("Received new marker location");
			marker_x = (double)msg->x;
			marker_y = (double)msg->y;
			marker_count += 1;
		}

	private:
		ros::NodeHandle n;
		ros::Publisher marker_pub;
		ros::Subscriber marker_sub;
		ros::Subscriber amcl_pose_sub;
		double marker_x;
		double marker_y;
		int marker_count = 0;
	};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");

	add_markers_pubsub add_markers;

	ros::spin();

	return 0;
  
}

