import sys,rclpy,select,tty, termios
from rclpy.node import Node
from autorobo_msgs.msg import Twistring

old_settings = termios.tcgetattr(sys.stdin)
send_cmd = ["c","p"]

def non_blocking_input():
    try:
        tty.setcbreak(sys.stdin.fileno())
        ready, _, _ = select.select([sys.stdin], [], [], 0.1)
        if ready:
            return sys.stdin.read(1)  # 1文字だけ読み込む
        return None
    except:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class teleop_key(Node):
    def __init__(self):
        super().__init__("teleop_key")
        self.pub=self.create_publisher(Twistring,"R1_control",10)
        
        self.timer = self.create_timer(0.05,self.cmd_send)
        self.twistring = Twistring()
        self.keys = set()
        self.nav_frag=True

    def cmd_send(self):
        key = non_blocking_input()
        if key!=None:
            self.twistring=Twistring()
            if key=="w":
                self.twistring.twist.linear.x=1.0
            elif key=="x":
                self.twistring.twist.linear.x=-1.0
            elif key=="a":
                self.twistring.twist.linear.y=1.0
            elif key=="d":
                self.twistring.twist.linear.y=-1.0
            elif key=="q":
                self.twistring.twist.angular.z=1.0
            elif key=="e":
                self.twistring.twist.angular.z=-1.0
            elif key=="o":
                self.twistring.cmd="ok"
            elif key=="n":
                self.nav_frag=not self.nav_frag
                self.twistring.cmd="n on" if self.nav_frag else "n off"
            elif key=="2":
                self.twistring.cmd="set c2"
            elif key=="3":
                self.twistring.cmd="set c3"
            elif key in send_cmd:
                self.twistring.cmd=key
            print("\033[7A")
            print("x: {: .3f}".format(self.twistring.twist.linear.x))
            print("y: {: .3f}".format(self.twistring.twist.linear.y))
            print("z: {: .3f}".format(self.twistring.twist.angular.z))
            print("\033[Kcmd: "+self.twistring.cmd)
            print("navigation: "+str(self.nav_frag))
            print("wasd:linear, left/right:angular, c:continue, p:pause, o:ok, n: navigation")
            self.pub.publish(self.twistring)

def main():
    print("start main")
    print("x:\ny:\nz:\ncmd:")
    print("navigation: True")
    print("wasd:linear, left/right:angular, c:continue, p:pause, o:ok, n: navigation")
    rclpy.init()
    node=teleop_key()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    print("\033[7A\033[Kend listening\n\n\n\n\n\n")


if __name__=="__main__":
    main()