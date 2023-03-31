#!/usr/bin/env python3

import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16
import tf



class Odom_Tf:
    
    def __init__(self):
        
        rospy.init_node("Odom_pub")
        
        self.rate= rospy.get_param('~rate',10.0) 
        
        self.wheel_radius = float(rospy.get_param('wheel_radius',0.0500))
        self.tick_per_meter = float(rospy.get_param('ticks_meter',955.414014))
        # self.base_width = float(rospy.get_param('~base_width', 0.30)) 
        self.wheel_separation_width= float(rospy.get_param('wheel_separation_width',0.415))
        self.wheel_separation_length=rospy.get_param('wheel_separation_length',0.415)
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') 
        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        
        
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        
        #encoder data previous
        
        self.old_ticks_front_left = None
        self.old_ticks_front_right = None
        self.old_ticks_rear_left = None
        self.old_ticks_rear_right = None
        
        
        
        #encoder data new
        self.new_left_front = 0
        self.new_right_front = 0
        self.new_left_rear = 0
        self.new_right_rear = 0
        #xy plane position
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        #xy speed 
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vth = 0.0
        
        self.lfmult = 0
        self.rfmult = 0
        self.lrmult = 0
        self.rrmult = 0
        
        self.prev_lfencoder = 0
        self.prev_rfencoder = 0
        self.prev_lrencoder = 0
        self.prev_rrencoder = 0
        
        
        #for plotting and testing
        # self.x_plotter=[]
        # self.y_plotter=[]
        # self.time_plotter=0
       
        
        self.then = rospy.Time.now()
        
        
        rospy.Subscriber("left_front_wheel",Int16,self.leftFrontwheelCallback)
        rospy.Subscriber("right_front_wheel",Int16,self.rightFrontwheelCallback)
        rospy.Subscriber("left_rear_wheel",Int16,self.leftRearwheelCallback)
        rospy.Subscriber("right_rear_wheel",Int16,self.rightRearwheelCallback)
        
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    def spin(self):
        Rate =rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.update()
            Rate.sleep()
            
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed =now - self.then
            self.then =now
            elapsed =elapsed.to_sec()
            
            if self.old_ticks_front_left == None and self.old_ticks_rear_left == None and self.old_ticks_front_right == None and self.old_ticks_rear_right == None :
                d_left_front =0
                d_right_front =0
                d_left_rear =0
                d_right_rear=0
            else:
                d_left_front=(self.new_left_front - self.old_ticks_front_left)/ self.tick_per_meter
                d_left_rear=(self.new_left_rear - self.old_ticks_rear_left)/ self.tick_per_meter
                d_right_front=(self.new_right_front - self.old_ticks_front_right)/ self.tick_per_meter
                d_right_rear=(self.new_right_rear - self.old_ticks_rear_right)/ self.tick_per_meter
            
            self.old_ticks_front_left = self.new_left_front
            self.old_ticks_front_right = self.new_right_front
            self.old_ticks_rear_left = self.new_left_rear 
            self.old_ticks_rear_right = self.new_right_rear 
            
            self.vel_wheel_front_left = d_left_front/elapsed
            self.vel_wheel_front_right = d_right_front/elapsed
            self.vel_wheel_rear_left = d_left_rear/elapsed
            self.vel_wheel_rear_right = d_right_rear/elapsed
            
            self.Vx = (self.vel_wheel_front_left+self.vel_wheel_front_right+self.vel_wheel_rear_left+self.vel_wheel_rear_right)*self.wheel_radius/4
            self.Vy = (-self.vel_wheel_front_left+self.vel_wheel_front_right+self.vel_wheel_rear_left-self.vel_wheel_rear_right)*self.wheel_radius/4
            self.Vth = (-self.vel_wheel_front_left+self.vel_wheel_front_right-self.vel_wheel_rear_left+self.vel_wheel_rear_right)*self.wheel_radius/(4*(self.wheel_separation_length+self.wheel_separation_width))
             
             
            dx= (self.Vx * cos(self.th) - self.Vy * sin(self.th)) *2
            dy=(self.Vx * sin(self.th) + self.Vy * cos(self.th)) *2
            dth=(self.Vth*4)
            
            self.x = self.x+dx
            self.y = self.y+dy
            self.th = self.th+dth
                    
            
            # print("move:position : x",self.x,"y",self.y)
           
            rotation = tf.transformations.quaternion_from_euler(0, 0, self.th)
          
            
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                rotation,
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.th))
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.Vx
            odom.twist.twist.linear.y = self.Vy
            odom.twist.twist.angular.z = self.Vth
            self.odomPub.publish(odom)
            
            
    def Odom_pub():
        pass     
    def cal_distance():
        pass
    def cal_velocity():
        pass     

           


    
    
    
    
    def leftFrontwheelCallback(self,msg):
     
        encoder =msg.data
        if(encoder < self.encoder_low_wrap and self.prev_lfencoder > self.encoder_high_wrap):
            self.lfmult =self.lfmult+1
        if(encoder > self.encoder_high_wrap and self.prev_lfencoder < self.encoder_low_wrap):
            self.lfmult=self.lfmult-1
        self.new_left_front = 1.0 * (encoder + self.lfmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lfencoder = encoder
    
    

    def rightFrontwheelCallback(self,msg):
        encoder =msg.data
        if(encoder < self.encoder_low_wrap and self.prev_rfencoder > self.encoder_high_wrap):
            self.rfmult =self.rfmult+1
        if(encoder > self.encoder_high_wrap and self.prev_rfencoder < self.encoder_low_wrap):
            self.rfmult=self.rfmult-1
        self.new_right_front = 1.0 * (encoder + self.rfmult * (self.encoder_max - self.encoder_min)) 
        self.prev_rfencoder = encoder
        
    def leftRearwheelCallback(self,msg):
     
        encoder = msg.data
        if(encoder < self.encoder_low_wrap and self.prev_lrencoder > self.encoder_high_wrap):
            self.lrmult = self.lrmult+1
        if(encoder > self.encoder_high_wrap and self.prev_lrencoder < self.encoder_low_wrap):
            self.lrmult = self.lrmult-1
        self.new_left_rear = 1.0 * (encoder + self.lrmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lrencoder = encoder
    
    

    def rightRearwheelCallback(self,msg):
        encoder = msg.data
        if(encoder < self.encoder_low_wrap and self.prev_rrencoder > self.encoder_high_wrap):
            self.rrmult = self.rrmult+1
        if(encoder > self.encoder_high_wrap and self.prev_rrencoder < self.encoder_low_wrap):
            self.rrmult = self.rrmult-1
        self.new_right_rear = 1.0 * (encoder + self.rrmult * (self.encoder_max - self.encoder_min)) 
        self.prev_rrencoder = encoder





if __name__ =='__main__':
    """ main """
    try:
        shinros_odom = Odom_Tf()
        shinros_odom.spin()
    except rospy.ROSInterruptException:
        pass
        
    






                
