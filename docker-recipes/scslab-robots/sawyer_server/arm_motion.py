import argparse
import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
#(source- https://github.com/RethinkRobotics/intera_sdk/blob/master/intera_examples/scripts/go_to_cartesian_pose.py)
from intera_motion_msgs.msg import TrajectoryOptions
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

import PyKDL
from tf_conversions import posemath

class arm(object):
    def __init__(self):
        # initialize our ROS node, registering it with the Master
        #rospy.init_node('Hello_Sawyer', anonymous=True)
        # create an instance of intera_interface's Limb class
        self.limb = intera_interface.Limb('right')
        self.tip_name="right_gripper_tip"
        # get the right limb's current joint angles
        self.ini_angles = self.limb.joint_angles()
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state()
        try:
            self.gripper = intera_interface.Gripper('right')
        except ValueError:
            rospy.logerr("Could not detect a gripper attached to the robot.")
            return
        # print the current joint angles
        print("Initialized sawyer instance with angles",self.ini_angles)
    def grip(self,offset):
        current = self.gripper.get_position()
        self.gripper.set_position(current - offset)
    def reset(self):        
        # move to neutral pose
        self.limb.move_to_neutral()
        # get the right limb's current joint angles now that it is in neutral
        self.angles = self.limb.joint_angles()
        print("Reset joint angles ")
        # print the current joint angles again
        #print("Reset joint angles to ",self.angles)
    def orient_handcam(self):
        print("Changing position so that hand camera can see properly")
        self.angles['right_j5']=-1.59
        wave = {'right_j6': self.angles['right_j6'], 'right_j5': self.angles['right_j5'], 'right_j4': self.angles['right_j4'], 'right_j3': self.angles['right_j3'], 'right_j2': self.angles['right_j2'], 'right_j1': self.angles['right_j1'], 'right_j0': self.angles['right_j0']}
        self.limb.move_to_joint_positions(wave,timeout=3.0,threshold=0.005)
        rospy.sleep(0.2)
    def orient_headcam(self):
        print("Changing position so that head camera can see properly")
        self.angles['right_j5']=0.569
        wave = {'right_j6': self.angles['right_j6'], 'right_j5': self.angles['right_j5'], 'right_j4': self.angles['right_j4'], 'right_j3': self.angles['right_j3'], 'right_j2': self.angles['right_j2'], 'right_j1': self.angles['right_j1'], 'right_j0': self.angles['right_j0']}
        self.limb.move_to_joint_positions(wave,timeout=3.0,threshold=0.005)
        rospy.sleep(0.2)
    def robot_locked_away(self):
        ret=False
        if(self.angles['right_j0']<-0.732 or self.angles['right_j0']>0.55):
            return True
        '''
        if(self.angles['right_j1']<-1.8 or self.angles['right_j1']>0.002):
            return True
        if(self.angles['right_j2']<-1.2 or self.angles['right_j2']>1.4):
            return True
        if(self.angles['right_j2']<-0.06 or self.angles['right_j2']>2.8):
            return True
        '''
        return ret
    def joint_angles(self):
        #j_a=[self.angles['right_j0'],self.angles['right_j1'],self.angles['right_j2'],self.angles['right_j3'],self.angles['right_j4'],self.angles['right_j5'],self.angles['right_j6']]
        cur_ang = self.limb.joint_angles()
        j_a = [cur_ang['right_j0'],cur_ang['right_j1'],cur_ang['right_j2'],cur_ang['right_j3'],cur_ang['right_j4'],cur_ang['right_j5'],cur_ang['right_j6']]
        j_a=np.reshape(j_a,[1,7])
        return j_a
    def move_arm(self,actions): #delta joint angles
        self.angles = self.limb.joint_angles()
        self.angles['right_j0']+=actions[0]
        self.angles['right_j1']+=actions[1]
        self.angles['right_j2']+=actions[2]
        self.angles['right_j3']+=actions[3]
        self.angles['right_j4']+=actions[4]
        self.angles['right_j5']+=actions[5]
        self.angles['right_j6']+=actions[6]
        wave = {'right_j6': self.angles['right_j6'], 'right_j5': self.angles['right_j5'], 'right_j4': self.angles['right_j4'], 'right_j3': self.angles['right_j3'], 'right_j2': self.angles['right_j2'], 'right_j1': self.angles['right_j1'], 'right_j0': self.angles['right_j0']}
        self.limb.move_to_joint_positions(wave,timeout=2.0,threshold=0.005)
        #rospy.sleep(0.2)
    def move_arm_to_pos(self,actions): #exact joint angles
        self.angles['right_j0']=actions[0]
        self.angles['right_j1']=actions[1]
        self.angles['right_j2']=actions[2]
        self.angles['right_j3']=actions[3]
        self.angles['right_j4']=actions[4]
        self.angles['right_j5']=actions[5]
        self.angles['right_j6']=actions[6]
        wave = {'right_j6': self.angles['right_j6'], 'right_j5': self.angles['right_j5'], 'right_j4': self.angles['right_j4'], 'right_j3': self.angles['right_j3'], 'right_j2': self.angles['right_j2'], 'right_j1': self.angles['right_j1'], 'right_j0': self.angles['right_j0']}
        self.limb.move_to_joint_positions(wave,timeout=2.0,threshold=0.01)
        #rospy.sleep(0.2)
    
    def halfway(self, a1,a2):
        t = {}
        t['right_j6'] = 0.5*(a2['right_j6']+a1['right_j6'])
        t['right_j5'] = 0.5*(a2['right_j5']+a1['right_j5'])
        t['right_j4'] = 0.5*(a2['right_j4']+a1['right_j4'])
        t['right_j3'] = 0.5*(a2['right_j3']+a1['right_j3'])
        t['right_j2'] = 0.5*(a2['right_j2']+a1['right_j2'])
        t['right_j1'] = 0.5*(a2['right_j1']+a1['right_j1'])
        t['right_j0'] = 0.5*(a2['right_j0']+a1['right_j0'])
        return t



    def move_arm_coord(self,tx,ty,tz, p = -3.1380447885495184, y = 0.005061184189791313, r = -3.1387344597288998): #Use inverse kinematics
        # p,y,r set to default overhead orientation
        '''
        overhead_orientation = Quaternion(
                             x=-0.00142460053167,
                             y=0.999994209902,
                             z=-0.00177030764765,
                             w=0.00253311793936)
        '''
        quat = quaternion_from_euler(p, y, r)
        overhead_orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])

        block_poses = list()
        # The Pose of the block in its initial location.
        # You may wish to replace these poses with estimates
        # from a perception node.
        block_poses.append(Pose(position=Point(x=tx, y=ty, z=tz),orientation=overhead_orientation))
        joint_angles = self.limb.ik_request(block_poses[0], self.tip_name)
        print("joint angles calculated ",joint_angles)
        if joint_angles==False:
            print("Specified end effector coordinate does not have an IK solution")
            #self.limb.set_joint_position_speed(0.001)
        else:
            #move to the joint angle in multiple steps to reduce jitter
            self.limb.set_joint_position_speed(0.3) #for some reason this doen not still affect the joint motion speed
            c1 = self.limb.joint_angles()
            c2 = joint_angles
            t = self.halfway(c1,c2)
            steps=10
            for _ in range(steps):
                self.limb.move_to_joint_positions(t,timeout=2.0,threshold=0.005)
                t = self.halfway(t,c2)
            
            #t2 = self.halfway(t1,c2)
            #self.limb.move_to_joint_positions(t2,timeout=2.0,threshold=0.005)
            
            self.limb.move_to_joint_positions(joint_angles,timeout=2.0,threshold=0.005)

    def move_arm_coord_nosim(self,tx,ty,tz, p = -3.1380447885495184, y = 0.005061184189791313, r = -3.1387344597288998): #Use inverse kinematics
        #source - https://github.com/RethinkRobotics/intera_sdk/blob/master/intera_examples/scripts/go_to_cartesian_pose.py
        #LOL ! this function only works in real robot because of some proprietary software shit
        #anyways the function is adapted here probably works on real robot but not in the gazebo sim
        #check out the 3rd last comment in this post- https://rethinkrobotics.interaforum.com/topic/300-gazebo-support/
        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = self.limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=0.1,
                                         max_linear_accel=0.1,
                                         max_rotational_speed=0.2,
                                         max_rotational_accel=0.2,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = self.limb)

        joint_names = self.limb.joint_names()
        endpoint_state = limb.tip_state('right')
        pose = endpoint_state.pose
        rot = PyKDL.Rotation.RPY(p,
                                 y,
                                 r)
        trans = PyKDL.Vector(tx,
                             ty,
                             tz)
        f2 = PyKDL.Frame(rot, trans)
        # and convert the result back to a pose message
        #(using pose relative to base frame of the robot)
        pose = posemath.toMsg(f2 * posemath.fromMsg(pose))
        poseStamped = PoseStamped()
        poseStamped.pose = pose
        #(let it figure out implicitly based on current joint angles of the robot)
        # using current joint angles for nullspace bais if not provided

        joint_angles = limb.joint_ordered_angles()
        waypoint.set_cartesian_pose(poseStamped, 'right', joint_angles)

        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=2.0)




            
    def get_xyz(self):
        c_end_pos = self.limb.endpoint_pose()
        c_val = c_end_pos.values()
        cex = c_val[0].x
        cey = c_val[0].y
        cez = c_val[0].z
        #print("x ",cex,"y ",cey,"z ",cez)
        return cex,cey,cez

def main():
    rospy.init_node('Hello_Sawyer', anonymous=True)
    am=arm()
    #cam=camera()
    am.reset()
    rospy.sleep(0.5)
    am.move_arm([0.5,0.5,0.5,0.5,0.5,0.5,0.5])

    am.move_arm([0.2,0.5,0.5,0.5,0.1,0.5,0.5])
    #rospy.spin()# If not there still everything runs, but outputs a huge rospy interruption exception on ctrl+c event
    
if __name__ == '__main__':
    main()
