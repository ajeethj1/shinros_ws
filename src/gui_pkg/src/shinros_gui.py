#!/usr/bin/env python3

""" ADD and REMOVE LOCATION in admin mode is under development 
    SQL for storing location detail is need to included"""

from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager
from kivy.factory import Factory
from kivy.uix.screenmanager import Screen
from kivy.clock import mainthread
from kivy.uix.popup import Popup
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
import time
import rospy
from functools import partial
import actionlib # Brings in the SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal # Brings in the .action file and messages used by the move base action
from geometry_msgs.msg import Pose

# Location detail add  
# sql need to be add for better data storing 
 
location_list=["3D printing division","Drone division","Robotic Division","Staff","Home"]
location_image=["image/3D_printing.jpg","image/Drone_division.jpg","image/Robot_division.jpg","image/staff.png","image/Home.png"]
location_orientation=[[5.2,-0.06,0.17,1.0],[12.8,-0.063,0.02063,1.0],[15.619,-0.0680,0.024699,1.0],[1.44515139,-6.9719842876,-0.70867978,0.705530],[0.0, 0.0, 0.0, 1]]

start=True
i=0

def setGoal(pose): 
   
    # client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose[0]
    goal.target_pose.pose.position.y = pose[1]
    goal.target_pose.pose.orientation.z = pose[2]
    goal.target_pose.pose.orientation.w = pose[3]


    client.send_goal(goal)
  
    # wait = client.wait_for_result()
  
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    
    #     return client.get_result()   
def stop_request():
    # cancel the target goal in client
    client.gh.cancel()
class WindowManager(ScreenManager):
    pass

class HomeWidget(Screen):
    pass
class AdminScreen(Screen):
    def status(self,task):
        self.popStatus=GridLayout(rows=2)

        
        self.label=Label(text="Successfully "+ task)
        self.closeButton=Button(text='Close',size_hint= (0.5, 0.5))
        self.popStatus.add_widget(self.label)
        self.popStatus.add_widget(self.closeButton)
        self.popupWindow = Popup(title="Status", content=self.popStatus, size_hint=(None,None),size=(200,200),auto_dismiss=True)
        
        self.popupWindow.open()
        self.closeButton.bind(on_press=self.popupWindow.dismiss)
    def add_location(self,name,description):
        pass
    def remove_location(self,name):
        pass
class Check(Screen):
    pass
class UserWidget(Screen):
    
    def adminCheck(self):
        self.show = Check()

        self.close_button=Button(text="close",size_hint=(0.9, 0.15) ,pos_hint= {"center_x":0.5, "y":0.1})
        self.show.add_widget(self.close_button)

        self.popupWindow = Popup(title="Admin login", content=self.show, size_hint=(None,None),size=(400,400),auto_dismiss=True)
        
        self.popupWindow.open()
        self.show.ids.submit.bind(on_press= self.pass_check)
        self.close_button.bind(on_press=self.popupWindow.dismiss)
    def pass_check(self,event):
        
        if self.show.ids.password.text == "vab":
            self.popupWindow.dismiss()
            self.parent.current = "Admin"
            
        else:
            self.popupWindow.title = "Failed login Retry!"
            
            self.popupWindow.dismiss()
            self.parent.current ="User"
class VisitorScreen(Screen):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.start=True
        
    def stop(self):
        stop_request()
        
        
    @mainthread
    def on_enter(self):
        if self.start is True:
            for i in range(len(location_list)):
                
                tile = Factory.MyTile(source=location_image[i],on_press=partial(self.callback,int(i)))

                tile.text = str("[size=26]"+location_list[i]+"[/size]")
                self.start=False
                self.ids.list_box.add_widget(tile)
            
            
    def callback(self, i,instance):
        setGoal(location_orientation[i])
    

class ShinrosApp(MDApp):
    def build(self):
        kv = Builder.load_file("shinros 1.0.kv")
        self.theme_cls.theme_style = "Dark"
        bg=self.theme_cls.bg_darkest
        # self.theme_cls.colors = colors
        self.theme_cls.primary_palette = "Cyan"
        self.theme_cls.accent_palette = "Teal"
        # self.theme_cls.primary_dark_hue:"200"
        return kv


if __name__ == "__main__":
    rospy.init_node('movebase_client_py')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    ShinrosApp().run()

        






    # ['Red', 'Pink', 'Purple', 'DeepPurple', 'Indigo', 'Blue', 'LightBlue', 'Cyan', 'Teal', 'Green', 'LightGreen', 'Lime', 'Yellow', 'Amber', 'Orange', 'DeepOrange', 'Brown', 'Gray', 'BlueGray']
