import os
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import Imu
import math


# throttle and direction for each wheel
THROTTLE_LEFT = 0.5  # 50% throttle
FORWARD = 1   # forward
THROTTLE_RIGHT = 0.5 # 30% throttle
BACKWARD = -1 # backward

class IMUTurnNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(IMUTurnNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        imu_topic = f"/{vehicle_name}/imu_node/data"
        
        # # form the message velocities
        # self._vel_left = 0
        # self._vel_right = 0
        # self._cnt = 0
        # self._prev_left_ticks = 0
        # self._prev_right_ticks = 0

        self.target_angle = 90.0
        self.current_angle = 0.0
        self.previous_time = None
        self.turning = True

        # construct publisher
        self.publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)
        

        # # Temporary data storage
        # self._ticks_left = None
        # self._ticks_right = None

        # self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        # self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # # Construct subscribers
        # self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        # self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)
        self.sub_imu = rospy.Subscriber(self.imu_topic, Imu, self.callback_imu)


    # def callback_left(self, data):
    #     # Log general information once at the beginning
    #     rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
    #     rospy.loginfo_once(f"Left encoder type: {data.type}")
        
    #     # Store data value
    #     self._ticks_left = data.data

    # def callback_right(self, data):
    #     # Log general information once at the beginning
    #     rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
    #     rospy.loginfo_once(f"Right encoder type: {data.type}")
        
    #     # Store data value
    #     self._ticks_right = data.data

    def callback_imu(self, data):
        if not self.turning:
            return
        current_time = rospy.Time.now()
        if self.previous_time is None:
            self.previous_time = current_time
            return
        dt = (current_time - self.previous_time).to_sec()
        self.previous_time = current_time
        
        angular_velocity_z = data.angular_velocity.z
        delta_angle = math.degrees(angular_velocity_z * dt)
        self.current_angle += delta_angle

        rospy.loginfo(f"Kąt obrotu: {self.current_angle:.2f}°")

        if abs(self.current_angle) >= self.target_angle:
            rospy.loginfo("Osiągnięto 90 stopni! Zatrzymanie.")
            self.stop_robot()
            self.turning = False
    
    def turn_robot(self, speed = 0.3):
        msg = WheelsCmdStamped()
        msg.vel_left = speed
        msg.vel_right = speed
        self.publisher.publish(msg)

    def stop_robot(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0
        msg.vel_right = 0
        self.publisher.publish(msg)

    def run(self):
        # rate = rospy.Rate(100)

        # while not rospy.is_shutdown():            
        #     if self._cnt < 100:
        #         self._vel_left = THROTTLE_LEFT * FORWARD
        #         self._vel_right = THROTTLE_RIGHT * FORWARD
        #         self._cnt += 1
        #         msg = f"Wheel encoder ticks LEFT, RIGHT: {self._ticks_left}, {self._ticks_right}"
                
        #     elif self._cnt < 150:
        #         self._vel_left = 0
        #         self._vel_right = 0
        #         self._cnt += 1
    
        #         self._prev_left_ticks = self._ticks_left
        #         self._prev_right_ticks = self._ticks_right
        #     else:
        #         self._vel_left = THROTTLE_LEFT * FORWARD * 0.5
        #         self._vel_right = THROTTLE_RIGHT * BACKWARD * 0.5

        #         if self._ticks_left is not None and self._ticks_right is not None:
        #             if(abs(self._prev_left_ticks - self._ticks_left) < round(730 / 4)):
        #                 msg = f"Wheel encoder ticks LEFT_PREV, LEFT, DIFF: {self._prev_left_ticks}, {self._ticks_left}, {(abs(self._prev_left_ticks - self._ticks_left))}"
        #             else:
        #                 self._cnt = 0
        #         else:
        #             self._cnt = 0

        #     message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        #     self._publisher.publish(message)
        #     rospy.loginfo(msg)
        #     rate.sleep()
        rospy.sleep(1)  # Poczekaj na inicjalizację IMU
        self.turn_robot()
        rospy.spin()  # Utrzymuje działanie węzła

    def on_shutdown(self):
        # Stop the wheels when the node shuts down
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)
        self.sub_left.unregister()
        self.sub_right.unregister()


if __name__ == "__main__":
    node_imu = IMUTurnNode(node_name='imu_turn_node')
    node_imu.run()