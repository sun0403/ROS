#!/usr/bin/python3
import sys
import os
import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from gazebo_msgs.msg import ModelStates
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
from pick_class import Pick


# Global variables
cube_pose = None
object_name = "cube_1"
updating_pose = True

# Define the target place position
place_position = Point(0.5, 0.5, 0.3)  # Example place position

def update_cube_pose_from_gazebo(msg):
    """Retrieve the pose of the specified cube from Gazebo."""
    global cube_pose

    if not updating_pose:
        return

    if object_name in msg.name:
        index = msg.name.index(object_name)
        cube_pose = PoseStamped()
        cube_pose.header.frame_id = "world"
        cube_pose.pose = msg.pose[index]

    

def main():
    """Main function to pick and place a single cube."""
    rospy.init_node('gazebo_to_moveit_single_pick_and_place_demo')

    # Subscribe to Gazebo's model_states topic to get the cube pose
    rospy.Subscriber("/gazebo/model_states", ModelStates, update_cube_pose_from_gazebo)
    rospy.loginfo("Subscribed to /gazebo/model_states to update cube pose.")

    # Wait for cube pose to be updated
    rospy.sleep(2.0)

    if not cube_pose:
        rospy.logerr("Failed to get cube pose from Gazebo. Exiting...")
        return

    # Initialize the Pick object
    pick_controller = Pick(object_name, cube_pose)

    # Execute the pick operation
    rospy.loginfo(f"Picking '{object_name}'...")
    pick_controller.execute_pick()

    # # Define the target place pose
    # place_pose = PoseStamped()
    # place_pose.header.frame_id = "world"
    # place_pose.pose.position = place_position
    # place_pose.pose.orientation = Quaternion(1, 0, 0, 0)  # Neutral orientation

    # # Execute the place operation
    # rospy.loginfo(f"Placing '{object_name}'...")
    # pick_controller.execute_place(place_pose)

    rospy.loginfo("Pick-and-place operation completed successfully.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted. Exiting...")
