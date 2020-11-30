#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <string>


class add_markers_pub {
	public:
		add_markers_pub() {
			marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		}
		
			// function to place or delete a visual marker
			void place_marker(double x, double y, std::string action) {

			// Marker type cube
		  uint32_t shape = visualization_msgs::Marker::CUBE;

			visualization_msgs::Marker marker;
			// Set the frame ID and timestamp.
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();

			// Set the namespace and id for this marker.
			marker.ns = "basic_shapes";
			marker.id = 0;

			// Set the marker type.
			marker.type = shape;

			// Set the marker action
			if (action == "add") {
				marker.action = visualization_msgs::Marker::ADD;
			}
			else if (action == "delete") {
				marker.action = visualization_msgs::Marker::DELETE;
			}

			// Set the pose of the marker
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

			// publish to the visualization_marker topic
			marker.lifetime = ros::Duration();
			marker_pub.publish(marker);
  	}

		void test() {
			
			// wait for subscriber to the "visualization_marker" topic
			while (marker_pub.getNumSubscribers() < 1) {
				// wait
			}
			ROS_INFO("Placing a marker");
			place_marker(6.0, 6.0, "add");
			ros::Duration(5.0).sleep();

			ROS_INFO("Deleting the marker");
			place_marker(6.0, 6.0, "delete");
			ros::Duration(5.0).sleep();

			ROS_INFO("Placing another marker");
			place_marker(2.0, -4.0, "add");
			ros::Duration(5.0).sleep();
		}

	private:
		ros::NodeHandle n;
		ros::Publisher marker_pub;
	};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");

	add_markers_pub add_markers;
	
	// publish a marker, wait five seconds, 
	// delete the marker, wait five seconds, publish another marker
	add_markers.test();

	ros::spin();

	return 0;
  
}

