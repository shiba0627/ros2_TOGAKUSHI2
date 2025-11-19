import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from sensor_msgs.msg import Joy
import readchar
import threading
import time

class doordata_subscriber(Node):
    def __init__(self):
        super().__init__('door_data_subscriber')
        #/door_dataを購読
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'door_data',
            self.door_data_callback,
            10
        )
        self.subscription
        self.siran = []
        self.pub_joy = self.create_publisher(Joy,'/whill/controller/joy', 10)
        self.coef_0_rotate = 0.1
        self.coef_1_forward = 0.1
        self.straight_flag = 0
        self.center_door = np.array([0.0, 0.0])
        # キーボード入力監視スレッド
        self.key_thread = threading.Thread(target=self.keyboard_control)
        self.key_thread.daemon = True
        self.key_thread.start()

    def door_data_callback(self, msg):
        #door_dataを受信
        if len(msg.data) != 5:
            self.get_logger().warn("受信した door_data のサイズが不正です")
            return
        data = msg.data
        #self.get_logger().info(f'data ={msg.data}')
        #self.get_logger().info(f'端点1:({data[0]},{data[1]})')
        #self.get_logger().info(f'端点2:({data[2]},{data[3]})')
        self.center_door = np.array([((data[0]+data[2])/2),((data[1]+data[3])/2)])
        #self.get_logger().info(f'中心点:{self.center_door}')
        self.siran.append()

        if self.center_door[1]<0:
            self.get_logger().info('左')
        elif self.center_door[1]>0:
            self.get_logger().info('右')

    def send_joy(self,axis_0_rotate, axis_1_foward):
        msg = Joy()
        axis_value = []
        axis_value.append(float(axis_0_rotate))
        axis_value.append(float(axis_1_foward))
        msg.axes = axis_value
        self.pub_joy.publish(msg)
    
    def keyboard_control(self): 
        while True:
            time.sleep(0.01)
            #print("teleop command -> forward:'w', backward:'x', left:'a', right:'d', stop:'s'")
            #print("teleop command -> speedUp:'+', speedDown:'-', exit:'q'")
            #print('自動制御: o')
            #print(f'door{self.center_door}')
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
            elif key == 'o':
                teisu = 70
                if self.straight_flag == 0:
                    if -teisu < self.center_door[1] <teisu:
                        axis_1_foward = 0.0
                        print('方向OK!!!')
                        self.straight_flag = 1
                    elif self.center_door[1] < -teisu:
                        axis_0_rotate = self.coef_0_rotate#右旋回
                    elif self.center_door[1] > teisu:
                        axis_0_rotate = -self.coef_0_rotate#左旋回
                elif self.straight_flag == 1:
                    print('直進！！')
                    axis_1_foward = self.coef_1_forward
                    self.send_joy(axis_0_rotate,axis_1_foward)
            else :
                axis_0_rotate = 0.0
                axis_1_foward = 0.0
                self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)
            self.send_joy(axis_0_rotate = axis_0_rotate, axis_1_foward = axis_1_foward)



def main():
    rclpy.init()
    node = doordata_subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()