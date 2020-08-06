#! /usr/bin/env python

import sys
import rospy

# Messages
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg

# Actions
import actionlib
import panda_unity_simulation.msg 


class MoveGroupInterface(object):
    """ A very basic move group interface for a fixed base manipulator"""
    # Plan path action 
    planPath_action_name = "plan_path_moveit"
    confirmPath_action_name = "confirm_path_moveit"
    
    def __init__(self, group_name, debug=False):
        super(MoveGroupInterface, self).__init__()

        self.debug = debug

        # Create a robot commander
        self.robot = moveit_commander.RobotCommander()

        # Instantiate planning scene
        self.scene = moveit_commander.PlanningSceneInterface()

        # Attach group commander to move group
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # Initialize plan and path plan action result
        self.plan = moveit_msgs.msg.RobotTrajectory() # empty trajectory
        self.planPath_action_result = panda_unity_simulation.msg.PlanPathMoveitResult()

        # Initialize plan path action server
        self.planPath_as = actionlib.SimpleActionServer(self.planPath_action_name, 
            panda_unity_simulation.msg.PlanPathMoveitAction, 
            execute_cb=self.PlanPathServerCb, 
            auto_start = False)

        # Initialize confirm path result
        self.confirmPath_action_result = panda_unity_simulation.msg.ConfirmPathMoveitResult()

        # Initialize confirm path action server
        self.confirmPath_as = actionlib.SimpleActionServer(self.confirmPath_action_name,
            panda_unity_simulation.msg.ConfirmPathMoveitAction,
            execute_cb=self.ConfirmPathServerCb,
            auto_start = False)

        # Show basic info about the manipulator upon start-up
        print("Manipulator info:")
        print("\tReference frame for planning: %s" % self.group.get_planning_frame())
        print("\tEnd effector link: %s" % self.group.get_end_effector_link())
        print("\tRobot groups: %s" % self.robot.get_group_names())

        # Start receiving requests
        self.planPath_as.start()
        self.confirmPath_as.start()

    # TODO: change this to client server paradigm
    def PlanPathServerCb(self, goal):
        """ PlanPath action server callback """
        # Set new target pose
        self.group.set_pose_target(goal.targetPose)
        # Plan - this will also show the trajectory in rviz
        self.plan = self.group.plan()
        # Check if trajectory is empty
        if self.plan.joint_trajectory.points:
            # Trajectory is not empty - result is true
            rospy.loginfo("New trajectory planned! Sending ack...")
            self.planPath_action_result.pathResult = True
        else:
            self.planPath_action_result.pathResult = False
        self.planPath_as.set_succeeded(self.planPath_action_result)

    def ConfirmPathServerCb(self,goal):
        """ Confirm path server callback"""
        # Execute previously planned path
        if(goal.confirmation):
            if(self.plan.joint_trajectory.points):
                status = self.group.execute(self.plan, wait=True)
                if(status):
                    self.confirmPath_action_result.confirmationResult = True
                else:
                    self.confirmPath_action_result.confirmationResult = False                    
            else:
                self.confirmPath_action_result.confirmationResult = False
        else:
            self.confirmPath_action_result.confirmationResult = False
        self.confirmPath_as.set_succeeded(self.confirmPath_action_result)
        
if __name__ == "__main__":
    try:
        # Initialize moveit commander - c++ under the hood
        moveit_commander.roscpp_initialize(sys.argv)
        # Initialize python node
        rospy.init_node("MoveGroupInterface")
        # Create an instance of MoveGroupInterface
        mgi = MoveGroupInterface("panda_arm", debug=True)
        # Keep node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass