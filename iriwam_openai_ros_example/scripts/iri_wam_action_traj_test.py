#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectoryPoint

class IriWamExecTrajectory(object):
    
    def __init__(self):
        
        """
        We initialise the Action client
        
        GOAL >>>>>>>
        trajectory_msgs/JointTrajectory trajectory
          std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
          string[] joint_names
          trajectory_msgs/JointTrajectoryPoint[] points
            float64[] positions
            float64[] velocities
            float64[] accelerations
            float64[] effort
            duration time_from_start
        control_msgs/JointTolerance[] path_tolerance
          string name
          float64 position
          float64 velocity
          float64 acceleration
        control_msgs/JointTolerance[] goal_tolerance
          string name
          float64 position
          float64 velocity
          float64 acceleration
        duration goal_time_tolerance
        
        
        These are the numbers for get_status
        class SimpleGoalState:
        PENDING = 0
        ACTIVE = 1
        DONE = 2
        WARN = 3
        ERROR = 4
        
        
        header: 
          seq: 0
          stamp: 
            secs: 502
            nsecs: 735000000
          frame_id: ''
        goal_id: 
          stamp: 
            secs: 502
            nsecs: 735000000
          id: "/iri_wam_reproduce_trajectory-1-502.735000000"
        goal: 
          trajectory: 
            header: 
              seq: 0
              stamp: 
                secs: 503
                nsecs: 734000000
              frame_id: "iri_wam_link_base"
            joint_names: [iri_wam_joint_1, iri_wam_joint_2, iri_wam_joint_3, iri_wam_joint_4, iri_wam_joint_5,
          iri_wam_joint_6, iri_wam_joint_7]
            points: 
              - 
                positions: [0.0112138, 0.942628, 0.133408, 1.65916, -0.214736, -1.05983, -0.430974]
                velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                effort: []
                time_from_start: 
                  secs: 1
                  nsecs:         0
              -
              
              ...
              
              - 
                positions: [-0.0989418, 1.00551, 0.263419, 1.45639, -0.514689, -0.903363, -0.349692]
                velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                effort: []
                time_from_start: 
                  secs: 12
                  nsecs: 560000000
            path_tolerance: []
            goal_tolerance: []
            goal_time_tolerance: 
            secs: 0
            nsecs:         0
            ---
        
        """
        
        # create the connection to the action server
        self.client = actionlib.SimpleActionClient('/iri_wam/iri_wam_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # waits until the action server is up and running
        self.client.wait_for_server()
        
        self.init_goal_message()
        
        joints_positions_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.send_joints_positions(joints_positions_array)
        
        
        
    
    def init_goal_message(self):
        """
        We initialise the variable that we will use to send the goals.
        We will reuse it because most of the values are fixed.
        """
        
        self.PENDING = 0
        self.ACTIVE = 1
        self.DONE = 2
        self.WARN = 3
        self.ERROR = 4
        
        # We Initialise the GOAL SYETS GOINT TO INIT POSE
        # creates a goal to send to the action server
        self.goal = FollowJointTrajectoryGoal()
        
        # We fill in the Goal

        
        self.goal.trajectory.header.stamp = rospy.Time.now()
        self.goal.trajectory.header.frame_id = "iri_wam_link_base"
        self.goal.trajectory.joint_names  = [   "iri_wam_joint_1",
                                                "iri_wam_joint_2",
                                                "iri_wam_joint_3",
                                                "iri_wam_joint_4",
                                                "iri_wam_joint_5",
                                                "iri_wam_joint_6",
                                                "iri_wam_joint_7"]
                                                
        self.goal.trajectory.points = []
        joint_traj_point = JointTrajectoryPoint()
        
        
        # TODO
        joint_traj_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.effort = []
        joint_traj_point.time_from_start = rospy.Duration.from_sec(1.0)
        
        self.goal.trajectory.points.append(joint_traj_point)
        
        
    def get_goal(self):
        return self.goal
        
        
        
    def feedback_callback(self, feedback):
        
        rospy.loginfo("##### FEEDBACK ######")
        #rospy.loginfo(str(feedback.joint_names))
        rospy.loginfo(str(feedback.desired.positions))
        #rospy.loginfo(str(feedback.actual.positions))
        #rospy.loginfo(str(feedback.error.positions))
        rospy.loginfo("##### ###### ######")
        
        
    def send_joints_positions(self, joints_positions_array):
        
        
        my_goal = self.get_goal()
        
        my_goal.trajectory.header.stamp = rospy.Time.now()
        joint_traj_point = JointTrajectoryPoint()
        
        
        # TODO
        joint_traj_point.positions = joints_positions_array
        joint_traj_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_traj_point.effort = []
        joint_traj_point.time_from_start = rospy.Duration.from_sec(1.0)
        
        my_goal.trajectory.points = []
        my_goal.trajectory.points.append(joint_traj_point)
        
        
        # sends the goal to the action server, specifying which feedback function
        # to call when feedback received
        self.client.send_goal(my_goal, feedback_cb=self.feedback_callback)
        
        # Uncomment these lines to test goal preemption:
        #self.client.cancel_goal()  # would cancel the goal 3 seconds after starting

        state_result = self.client.get_state()

        rate = rospy.Rate(10)
        
        rospy.loginfo("state_result: "+str(state_result))
        
        while state_result < self.DONE:
            rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
            rate.sleep()
            state_result = self.client.get_state()
            rospy.loginfo("state_result: "+str(state_result))
            
        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == self.ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == self.WARN:
            rospy.logwarn("There is a warning in the Server Side")



if __name__ == '__main__':

    rospy.init_node('iriwam_exec_traj_test_node', anonymous=True, log_level=rospy.INFO)
    traj_object = IriWamExecTrajectory()
    
    rospy.logwarn("Sening NOn ZERO trajectory")
    joints_positions_array = [1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    traj_object.send_joints_positions(joints_positions_array)
    
    rospy.spin()
    