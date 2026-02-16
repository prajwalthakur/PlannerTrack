#!/usr/bin/env python3
import os
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
from ament_index_python.packages import get_package_share_directory

########
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from rclpy.duration import Duration

from project_utils.msg import EigenVector

#print(matplotlib.get_backend())  # Should output "QtAgg"
# matplotlib.use("Agg")  # Explicitly set the backend
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import matplotlib.patches as patches



#####################
class visualizer_class(Node):
    def __init__(self):
        super().__init__("visualizer_node")
        self.node_init()
        self.state_sub  = self.create_subscription(EigenVector,'/ego_state',self.state_sub_callback,10)
        self.state_sub
        self.timer = self.create_timer(self.plot_update_dt,self.plot_cb)
        
    def node_init(self):
        # Declare the parameters you expect
        self.declare_parameter('map_dir', '')
        self.declare_parameter('veh_width',0.0)
        self.declare_parameter('veh_length',0.0)
        self.declare_parameter('default_x_pos',0.0)
        self.declare_parameter('default_y_pos',0.0)
        self.declare_parameter('default_yaw',0.0)
        self.declare_parameter('map','')
        self.declare_parameter('plot_update_dt',0.0)
        self.declare_parameter('x_lim',0.0)
        self.declare_parameter('y_lim',0.0)
        
        self.map_dir = self.get_parameter('map_dir').get_parameter_value().string_value
        
        
        self.veh_length = self.get_parameter('veh_length').get_parameter_value().double_value
        self.get_logger().info(f'** veh_length parameter = "{self.veh_length}" **')  
        
        self.veh_width = self.get_parameter('veh_width').get_parameter_value().double_value
        self.ego_x_pose = self.get_parameter('default_x_pos').get_parameter_value().double_value
        self.ego_y_pose = self.get_parameter('default_y_pos').get_parameter_value().double_value
        self.ego_yaw = self.get_parameter('default_yaw').get_parameter_value().double_value
        self.map = self.get_parameter('map').get_parameter_value().string_value
        self.plot_update_dt = self.get_parameter('plot_update_dt').get_parameter_value().double_value
        self.x_lim = self.get_parameter('x_lim').get_parameter_value().double_value
        self.y_lim = self.get_parameter('y_lim').get_parameter_value().double_value
        
        
        # get the path to the map
        self.map_path = os.path.join(self.map_dir,self.map)
        self.get_logger().info(f'** map_path parameter = "{ self.map_path}" **')

        self.robot_fig , self.robot_axis = plt.subplots()
        self.robot_axis.set_xlim( -self.x_lim , self.x_lim   )
        self.robot_axis.set_ylim( -self.y_lim , self.y_lim )
        self.robot_axis.set_aspect('equal')
        self.robot_fig.tight_layout()
        self.plot_track(self.robot_axis , self.robot_fig)

        vertex_x , vertex_y  = self.calc_vertex()  
        vertex_directions = np.hstack((np.array(vertex_x).reshape((4,1)),np.array(vertex_y).reshape((4,1))))
        self.robot_state = patches.Polygon(
                vertex_directions,
                alpha=1.0,
                closed=True,
                fc='b',
                ec="None",
                zorder=10,
                linewidth=2,
            )        
        # Add grid
        self.robot_axis.grid(
            True,  # Enable grid
            which='both',  # Major and minor grid lines
            linestyle='--',  # Dashed lines
            linewidth=0.5,  # Line width
            alpha=0.7  # Transparency of grid lines
        )
        self.robot_axis.add_patch(self.robot_state)    
             
    def calc_vertex(self):
        l = self.veh_length / 2
        w = self.veh_width / 2
        vertex_x = [
            self.ego_x_pose + l * np.cos(self.ego_yaw) - w * np.sin(self.ego_yaw),
            self.ego_x_pose + l * np.cos(self.ego_yaw) + w * np.sin(self.ego_yaw),
            self.ego_x_pose - l * np.cos(self.ego_yaw) + w * np.sin(self.ego_yaw),
            self.ego_x_pose - l * np.cos(self.ego_yaw) - w * np.sin(self.ego_yaw),
        ]
        vertex_y = [
                    self.ego_y_pose + l * np.sin(self.ego_yaw) + w * np.cos(self.ego_yaw),
                    self.ego_y_pose + l * np.sin(self.ego_yaw) - w * np.cos(self.ego_yaw),
                    self.ego_y_pose - l * np.sin(self.ego_yaw) - w * np.cos(self.ego_yaw),
                    self.ego_y_pose - l * np.sin(self.ego_yaw) + w * np.cos(self.ego_yaw),
                ]
        return vertex_x , vertex_y        
        
    def set_lines(self,col,waypoints, ax , fig,line_size=1 ):
        track_line =  lines.Line2D([],[],linestyle='--',color = col ,linewidth=line_size)
        track_line.set_data(waypoints[:,0],waypoints[:,1])
        ax.add_line(track_line)  
        
    def plot_track(self,ax,fig):
        waypoints =  np.loadtxt(self.map_path, delimiter=',')
        waypoints_center  = waypoints[:,0:2] 
        #self.track_object.dict_waypoints
        # waypoints_inner,waypoints_center,waypoints_outer = self.track_object.get_coordinates(waypoints)                 
        # range_waypoints = len(waypoints_center)-1
        # self.set_lines('b',waypoints_inner , ax , fig )
        self.set_lines('r',waypoints_center, ax , fig ,line_size=0.4)
        #self.set_lines('g',waypoints_outer, ax , fig )        
        
      
    def update_poses(self,poses):
        self.drone_poses = []
        #self.drone_poses = poses
        # for i in range(0,self.num_drones):
        #     self.drone_poses.append([poses[i].data[0], poses[i].data[1] ,poses[i].data[2] ])
        self.ego_x_pose =     poses[0]
        self.ego_y_pose = poses[1]
        self.ego_yaw =  poses[2]
    
    def state_sub_callback(self,msg):
        
        poses =  msg.data   # NP array
        self.update_poses(poses)            
          
    def plot_cb(self):
        #poses = self.drone_poses
        #self.update_ellipsoids(poses,self.a_drone,self.b_drone,self.c_drone)
        # self.ego_x_history.append(self.ego_x_pose)
        # self.ego_y_history.append(self.ego_y_pose)
        vertex_x , vertex_y  = self.calc_vertex()
        
        self.robot_state.set_xy( np.array([vertex_x, vertex_y]).T  )  
        self.camera_center = [self.ego_x_pose, self.ego_y_pose]
        # Update plot limits based on camera center and car dimensions
        buffer = 2 # Add a buffer around the car for better visibility
        xlim_min = self.camera_center[0] - self.veh_length/2 - 2*buffer
        xlim_max = self.camera_center[0] + self.veh_length/2 + 2*buffer
        ylim_min = self.camera_center[1] - self.veh_width/2 - buffer
        ylim_max = self.camera_center[1] + self.veh_width/2 + buffer
        self.robot_axis.set_xlim(xlim_min, xlim_max)
        self.robot_axis.set_ylim(ylim_min, ylim_max)        
        
        
        self.robot_fig.canvas.draw_idle()        # schedule a repaint
        self.robot_fig.canvas.flush_events()     # force the GUI to process the redraw
   
          
    def spin(self):
        plt.ion()
        self.robot_fig.show()
        self.get_logger().info("vehicle_sim_node spinning")
        # plt.show()
        rclpy.spin(self)
        plt.close(self.robot_fig)

            

def main():
    #plt.switch_backend("Qt4Agg")
    rclpy.init()
    visualizer_class_node = visualizer_class()
    visualizer_class_node.spin()
    rclpy.shutdown()

if __name__=='__main__':
    main()
    
    
    

