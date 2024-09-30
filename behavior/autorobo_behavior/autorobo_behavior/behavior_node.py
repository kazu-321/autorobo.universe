import rclpy
from rclpy.node import Node
from autorobo_msgs.msg import Twistring
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int8
from std_msgs.msg import String
import math

class BTNode(Node):
    def __init__(self):
        super().__init__('behavior_node')
        # pub/sub
        self.R1_pub  = self.create_publisher(Twistring, '/R1', 10)
        self.R1_sub  = self.create_subscription(Twistring, '/R1_control', self.R1_callback, 10)
        self.nav_sub = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.msg_pub = self.create_publisher(String, '/behavior/message', 10)
        self.int_pub = self.create_publisher(Int8, '/behavior/process', 10)
        self.goal_pub= self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.err_sub = self.create_subscription(Twist, '/planning/error', self.err_callback, 10)
        self.twi_pub = self.create_publisher(TwistStamped, '/cmd_view',10)
        self.loop    = self.create_timer(0.05,self.loop_callback)

        # variables
        self.nav_frag= True
        self.reached = False
        self.forward = False
        self.team    = "red"
        self.process = 0
        self.counter = -1
        self.cmd     = Twistring()
        self.err     = Twist()
        self.vel_nav = Twist()
        self.c2      = 0.0
        self.c3      = 0.0
        self.declare_parameter("pos_tol",35.0)
        self.declare_parameter("ang_tol",10.0)
        self.pos_tol = self.get_parameter("pos_tol").get_parameter_value().double_value
        self.ang_tol = self.get_parameter("ang_tol").get_parameter_value().double_value
        self.command = ""
        self.commands= []
        self.bt=[]
        self.bt_msg=[]

        self.declare_parameter("bt_file","")
        self.load_bt()
    
    
    def load_bt(self):
        self.bt_file = self.get_parameter("bt_file").get_parameter_value().string_value
        if self.bt_file=="":
            self.get_logger().error("No BT file specified")
            exit()
        with open(self.bt_file) as f:
            for line in f:
                if(line.strip()!="" and line.strip()[0:2]!="//"):
                    bt=line.split("//")
                    self.bt.append(bt[0].rstrip())
                    if(len(bt)>1):
                        self.bt_msg.append(bt[1].rstrip())
                    else:
                        self.bt_msg.append(bt[0].rstrip())
        self.bt.append("end")
        self.process=0

    def R1_callback(self, msg):
        self.cmd = msg
        cmds=msg.cmd.split(" ")
        if(msg.cmd=="n on"):
            self.nav_frag = True
        elif(msg.cmd=="n off"):
            self.nav_frag = False
        elif(msg.cmd=="ok"):
            if(self.command=="wait_ok"):
                self.process += 1
        elif(cmds[0]=="team"):
            self.team = cmds[1]
        elif(cmds[0]=="set"):
            if(cmds[1]=="c2"):
                self.c2 = float(cmds[2])
            elif(cmds[1]=="c3"):
                self.c3 = float(cmds[2])
            elif(cmds[1]=="process"):
                if(cmds[2]=="zero"):
                    self.process = 0
                    self.forward = False
                elif(cmds[2]=="charge"):
                    self.process = 15
                    self.forward = False
                elif(cmds[2]=="c2"):
                    self.process = 15
                    self.forward = False
                elif(cmds[2]=="c3"):
                    self.process = 22
                    self.forward = False

        self.send_cmd(self.cmd.cmd)
    
    def nav_callback(self, msg):
        self.vel_nav = msg
    
    def err_callback(self, msg):
        self.err = msg
        self.pos_tol = self.get_parameter("pos_tol").get_parameter_value().double_value
        self.ang_tol = self.get_parameter("ang_tol").get_parameter_value().double_value
        if(self.commands[0]=="goto" and abs(self.err.linear.x)*100<self.pos_tol and abs(self.err.linear.y)*100<self.pos_tol and abs(self.err.angular.z/math.pi*180)<self.ang_tol):
            self.get_logger().info("Reached err: ("+str(self.err.linear.x)+","+str(self.err.linear.y)+")")
            self.process += 1
    
    def loop_callback(self):
        self.command=self.bt[self.process]
        self.commands=self.command.split(" ")
        if(self.command=="end"):
            self.process=0
            return
    
        if(self.commands[0]=="wait"):
            if(self.counter==-1):
                self.counter = 0
            self.counter += 1
            if(self.counter>=float(self.commands[1])*20):
                self.counter = -1
                self.process += 1

        if(self.commands[0]=="goto"):
            if(self.commands[1]=="home"):
                if(self.team=="red"):
                    self.send_goal(0.0,0.0)
                elif(self.team=="blue"):
                    self.send_goal(0.0,3.7)
            elif(self.commands[1]=="c2"):
                self.send_goal(1.8,self.c2)
            elif(self.commands[1]=="c3"):
                self.send_goal(1.8,self.c3)
            else:
                x = float(self.commands[1])
                y = float(self.commands[2])
                self.send_goal(x, y)
        
        if(self.commands[0]=="send"):
            cmd = " ".join(self.commands[1:])
            self.send_cmd(cmd)
            self.process += 1
        
        if(self.commands[0]=="forward"):
            if(self.commands[1]=="on"):
                self.forward = True
            elif(self.commands[1]=="off"):
                self.forward = False
            self.process += 1
        
        if(self.nav_frag):
            self.cmd.twist = self.vel_nav

        if(self.forward):
            self.cmd.twist.linear.x = 0.25
            self.cmd.twist.linear.y = 0.0
            self.cmd.twist.angular.z = 0.0

        self.send_twist(self.cmd.twist)

        msg = String()
        msg.data = str(self.process+1)+"."+self.bt_msg[self.process]+"  ("+self.command+", t:"+self.team[0]+", c2:"+str(self.c2)+", c3:"+str(self.c3)+")"
        self.msg_pub.publish(msg)

        msg = Int8()
        msg.data = self.process+1
        self.int_pub.publish(msg)

        twi = TwistStamped()
        twi.twist = self.cmd.twist
        twi.header.frame_id="twi_view"
        twi.header.stamp=self.get_clock().now().to_msg()
        self.twi_pub.publish(twi)
    
    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.pose.position.x = x
        goal.pose.position.y = y
        self.goal_pub.publish(goal)
        self.err.linear.x = 100.0
        self.err.linear.y = 100.0
    
    def send_cmd(self, cmd):
        msg=Twistring()
        msg.cmd = cmd
        self.R1_pub.publish(msg)

    def send_twist(self, twist):
        msg = Twistring()
        msg.twist = twist
        self.R1_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bt_node = BTNode()
    rclpy.spin(bt_node)
    bt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()