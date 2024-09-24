import sys,rclpy,threading,tty,termios,select
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from autorobo_msgs.msg import Twistring
from pynput import keyboard

kb1=list("wasdWASD")                # 長押し対応キー
kb2=["right","left"]    # 長押し特殊キー
kb3=list("cpet")                    # 離し時対応キー


def non_blocking_input():
    old_settings = termios.tcgetattr(sys.stdin)
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


class pcNode(Node):
    def __init__(self):
        super().__init__("kb_controller")
        self.pub=self.create_publisher(Twistring,"R1_control",10)
        self.timer = self.create_timer(0.01,self.call)
        self.inputs= self.create_timer(0.01,self.input_callback)
        self.checker= self.create_timer(1,self.check)
        self.twistring=Twistring()
        self.keys=set()
        self.do_nav=False
        self.send=[]
        self.start()

    def check(self):
        datas=self.get_subscriptions_info_by_topic("cmd")
        if "_CREATED_BY_BARE_DDS_APP_" in [data._node_name for data in datas]:
            print("\033[9;0H\033[K767:o")
        else:
            print("\033[9;0H\033[K767:x")
        print("\033[9;7H"+str(self.do_nav))


    def call(self):
        global nav_speed
        self.twistring.twist.linear.x=0.0
        self.twistring.twist.linear.y=0.0
        self.twistring.twist.angular.z=0.0
        self.twistring.cmd=""

        for key in self.keys:
            if key=="w":
                self.twistring.twist.linear.x+=1.0
            elif key=="a":
                self.twistring.twist.linear.y+=1.0
            elif key=="s":
                self.twistring.twist.linear.x-=1.0
            elif key=="d":
                self.twistring.twist.linear.y-=1.0
            elif key=="right":
                self.twistring.twist.angular.z-=1.0
            elif key=="left":
                self.twistring.twist.angular.z+=1.0
        
        print("\033[6;0H\033[Kx:"+str(self.twistring.twist.linear.x))
        print("\033[7;0H\033[Ky:"+str(self.twistring.twist.linear.y))
        print("\033[8;0H\033[Kz:"+str(self.twistring.twist.angular.z))
        self.pub.publish(self.twistring)


    def input_callback(self):
        print("\033[5;1H\033[K")
        print("\033[4;1H\033[Kcmd>>> "+"".join(self.send))
        key=non_blocking_input()
        while key!=None:
            print(key)
            if key=="\n":
                self.twistring.cmd="".join(self.send)
                print("\033[3;1H\033[Ksend command:"+self.twistring.cmd)
                self.pub.publish(self.twistring)
                self.send=[]
            elif ord(key)==127:
                try:
                    self.send.pop()
                except:
                    pass
            else:
                self.send.append(key)
            key=non_blocking_input()

    def on_press(self,key):
        try:
            key=str(key.char)
        except AttributeError:
            key =str(key)[4:]
        if key in kb1:
            self.keys.add(key.lower())
        if key in kb2:
            self.keys.add(key)
        self.show()


    def on_release(self,key):
        try:
            key=str(key.char)
        except AttributeError:
            key=str(key)[4:]
        try:
            if key in kb1:
                self.keys.remove(key.lower())
            elif key in kb2:
                self.keys.remove(key)
            elif key in kb3:
                self.twistring.cmd=key
                self.pub.publish(self.twistring)
            elif key=="n":
                self.do_nav=not self.do_nav
                if self.do_nav:
                    self.twistring.cmd="n on"
                else:
                    self.twistring.cmd="n off"
                self.pub.publish(self.twistring)
            elif key=="o":
                self.twistring.cmd="ok"
                self.pub.publish(self.twistring)
        except:
            pass
        self.show()


    def show(self):
        print("\033[2;10H\033[K"+",".join(self.keys))


    def start(self):
        print("\033[1;1H\033[Kstart listening\n")
        self.listener=keyboard.Listener(on_press=self.on_press,on_release=self.on_release)
        self.listener.start()

def main():
    print("\033[1;1H\033[Kstart main")
    print("\033[2;1H\033[Kpressing:")
    print("\033[3;1H\033[Ksend command:")
    print("\033[4;1H\033[Kcmd>>>")
    rclpy.init()
    node=pcNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass
    print("\033[1;1H\033[Kend listening\n\n\n")


if __name__=="__main__":
    main()