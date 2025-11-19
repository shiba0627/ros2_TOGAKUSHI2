
# for ros2
import rclpy
from rclpy.node import Node
# sensor_msgs/msg/Joy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import readchar
import socket


class TeleopJoyPublisher(Node):
  def __init__(self):

    super().__init__('teleop_joy_publisher_node')

    #self.pub_joy = self.create_publisher(Joy,'/whill/controller/joy', 10)
    self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    self.twist_msg = Twist()
    self.twist_msg.linear.x = 0.2
    self.twist_msg.angular.z = 0.0
    #self.coef_0_rotate = 0.1
    #self.coef_1_forward = 0.1

    timer_period = 0.01  # 秒
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    
    
    host_ip = '192.168.1.102'
    host_port = 12346

    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    self.server_socket.bind((host_ip, host_port))

  def send_joy(self,axis_0_rotate, axis_1_foward):
      msg = Joy()
      axis_value = []
      axis_value.append(float(axis_0_rotate))
      axis_value.append(float(axis_1_foward))
      msg.axes = axis_value
      self.pub_joy.publish(msg)
      
  def Socket(self):
    '''
    host_ip = '192.168.1.102'
    host_port = 12346

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.bind((host_ip, host_port))
    '''

    self.server_socket.listen(1)

    print("待機中")
    
    client_socket, client_address = self.server_socket.accept()

    data = client_socket.recv(1024)
    
    print("受信メッセージ：", data.decode())
    return data.decode()

  def timer_callback(self): 

    print("teleop command -> forward:'w', backward:'x', left:'a', right:'d', stop:'s'")
    print("teleop command -> speedUp:'+', speedDown:'-', exit:'q'")
    
    self.cmd_vel_pub.publish(self.twist_msg)
    '''
    key = self.Socket()
    if key == 'w':
        self.twist_msg.linear.x = 0.1
        self.cmd_vel_pub.publish(self.twist_msg)
        print("w")
    elif key == 'x':
        #msg.front = back_val
        axis_1_foward = - self.coef_1_forward
        print("x")
    elif key == 'a':
        #msg.side = left_val
        axis_0_rotate = self.coef_0_rotate
        print("a")
    elif key == 'd':
        #msg.side = right_val
        axis_0_rotate = - self.coef_0_rotate
        print("d")
    elif key == "+":
        if self.coef_0_rotate < 0.5 :
          self.coef_0_rotate = self.coef_0_rotate + 0.1
        if self.coef_1_forward < 0.5 :
          self.coef_1_forward = self.coef_1_forward + 0.1
        print("coef_0_rotate" + str(self.coef_0_rotate))
        print("coef_1_forward" + str(self.coef_1_forward))
 
    elif key == "-":
        if self.coef_0_rotate > 0.0 :
          self.coef_0_rotate = self.coef_0_rotate - 0.1
        if self.coef_1_forward > 0.0 :
          self.coef_1_forward = self.coef_1_forward - 0.1
        print("coef_0_rotate" + str(self.coef_0_rotate))
        print("coef_1_forward" + str(self.coef_1_forward))
 
    elif key == 's':
      axis_0_rotate = 0.0
      axis_1_foward = 0.0
      print("s")

    elif key == 'q':
      axis_0_rotate = 0.0
      axis_1_foward = 0.0
      self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
      exit()

    else :
      axis_0_rotate = 0.0
      axis_1_foward = 0.0
      self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
 

    '''
    #self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
    '''
    axis_0_rotate = 0.0
    axis_1_foward = 0.0
    key = readchar.readchar()
    
    if key == 'w':
        #msg.front = front_val
        axis_1_foward = self.coef_1_forward
        self.send_joy(axis_0_rotate,axis_1_foward)
        print("w")
    elif key == 'x':
        #msg.front = back_val
        axis_1_foward = - self.coef_1_forward
        print("x")
    elif key == 'a':
        #msg.side = left_val
        axis_0_rotate = self.coef_0_rotate
        print("a")
    elif key == 'd':
        #msg.side = right_val
        axis_0_rotate = - self.coef_0_rotate
        print("d")
    elif key == "+":
        if self.coef_0_rotate < 0.5 :
          self.coef_0_rotate = self.coef_0_rotate + 0.1
        if self.coef_1_forward < 0.5 :
          self.coef_1_forward = self.coef_1_forward + 0.1
        print("coef_0_rotate" + str(self.coef_0_rotate))
        print("coef_1_forward" + str(self.coef_1_forward))
 
    elif key == "-":
        if self.coef_0_rotate > 0.0 :
          self.coef_0_rotate = self.coef_0_rotate - 0.1
        if self.coef_1_forward > 0.0 :
          self.coef_1_forward = self.coef_1_forward - 0.1
        print("coef_0_rotate" + str(self.coef_0_rotate))
        print("coef_1_forward" + str(self.coef_1_forward))
 
    elif key == 's':
      axis_0_rotate = 0.0
      axis_1_foward = 0.0
      print("s")

    elif key == 'q':
      axis_0_rotate = 0.0
      axis_1_foward = 0.0
      self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
      exit()

    else :
      axis_0_rotate = 0.0
      axis_1_foward = 0.0
      self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
 


    self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
    '''
def main(args=None):

    rclpy.init(args=args)

    #teleop_pub = rospy.Publisher('whill_setjoystick', msgWhillSetJoystick, queue_size=10)
    #node = Node('keyboard_to_joy_node')
    #teleop_pub = node.create_publisher(Joy, '/whill/controller/joy', 10)
    teleop_joy_publisher = TeleopJoyPublisher()


    rclpy.spin(teleop_joy_publisher)
    
    teleop_joy_publisher.client_socket.close()
    teleop_joy_publisher.server_socket.close()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

