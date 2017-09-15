#!/usr/bin/env python
import threading
import rospy
import sensor_msgs.msg

class JointStateWrapper:
    def __init__(self):
        self.mutex = threading.Lock()
        self.M_PI = 3.14159265358979
        #self.FINGER_SCALE = 50.0/180.0*self.M_PI/110.0 # 50 is max finger joint angel, 110 is wpi return max rad
        self.FINGER_SCALE = 0.00793331478179

        # init joint state msg
        self.msg = sensor_msgs.msg.JointState()
        self.msg_seq = 0
        self.initMsg()

        self.arm_sub = rospy.Subscriber("jaco_arm/joint_states", sensor_msgs.msg.JointState, self.ArmCB)
        self.pub = rospy.Publisher("joint_states", sensor_msgs.msg.JointState, queue_size=10)

    def initMsg(self):
        self.ARM_DOF=6
        self.FINGER_DOF = 3
        #self.MOBILE_AXIS_DOF=4

        #self.ARM_OFFSET = self.MOBILE_AXIS_DOF

        #self.msg.name.append("world_to_robot_x")
        #self.msg.position.append(0.0)
        #self.msg.velocity.append(0.0)
        #self.msg.effort.append(0.0)

        #self.msg.name.append("world_to_robot_y")
        #self.msg.position.append(0.0)
        #self.msg.velocity.append(0.0)
        #self.msg.effort.append(0.0)

        #self.msg.name.append("world_to_robot_roll")
        #self.msg.position.append(0.0)
        #self.msg.velocity.append(0.0)
        #self.msg.effort.append(0.0)

        #self.msg.name.append("support_to_base")
        #self.msg.position.append(0.0)
        #self.msg.velocity.append(0.0)
        #self.msg.effort.append(0.0)


        for j in range(self.ARM_DOF):
            self.msg.name.append("jaco_joint_"+str(j+1)) # j+1 because begin from 1
            self.msg.position.append(0.0)
            self.msg.velocity.append(0.0)
            self.msg.effort.append(0.0)
        for k in range(self.FINGER_DOF):
            self.msg.name.append("jaco_joint_finger_"+str(k+1)) # k+1 because begin from 1
            self.msg.position.append(0.0)
            self.msg.velocity.append(0.0)
            self.msg.effort.append(0.0)

    def ArmCB(self, msg):
        self.mutex.acquire()
        if self.msg_seq==0:
            self.msg.header = msg.header
        self.msg_seq += 1
        self.msg.header.seq = self.msg_seq
        self.msg.header.stamp  = rospy.Time.now()

        # sort joints
        joint_tuples = []
        for i in range(len(msg.name)):
            if i==6 or i== 7 or i==8 :
                print 'origin version: ', msg.position[i]
                tp = (msg.name[i], msg.position[i]*self.FINGER_SCALE, msg.velocity[i]*self.FINGER_SCALE, msg.effort[i])
                print 'rad: ', tp[1]
                print 'angle: ', tp[1]/self.M_PI*180
                joint_tuples.append(tp)
            else:
                tp = (msg.name[i], msg.position[i], msg.velocity[i], msg.effort[i])
                joint_tuples.append(tp)
        sorted(joint_tuples, key=lambda joint: joint[0])
        
        #offset = self.ARM_OFFSET


        for i in range(len(joint_tuples)):
            self.msg.position[i] = joint_tuples[i][1]
            self.msg.velocity[i] = joint_tuples[i][2]
            self.msg.effort[i] = joint_tuples[i][3]

        self.pub.publish(self.msg)
        self.mutex.release()


if __name__=="__main__":
    rospy.init_node("tester")
    test = JointStateWrapper()
    rospy.spin()
