#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy import interpolate
from scipy import signal
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        ### PARAMETERS ###
        traj_csv = "spline.csv"#"traj_race_cl.csv"  # Name of csv with trajectory points
        self.sim_flag = True  # Set flag true for simulation, false for real

        # TODO: create ROS subscribers and publishers
        if self.sim_flag:
            self.pose_subscriber = self.create_subscription(Odometry, 'ego_racecar/odom', self.pose_callback, 1)
        else:
            self.pose_subscriber = self.create_subscription(Odometry, 'pf/pose/odom', self.pose_callback, 1)

        #For vizualization
        self.goal_points_publisher = self.create_publisher(MarkerArray, 'pp_goal_points_rviz',1)
        self.current_goal_publisher = self.create_publisher(PointStamped, 'pp_current_goal_rviz',1)
        self.spline_publisher = self.create_publisher(Marker, 'pp_spline_rviz',1)

        #For drive control
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 1)

        #For talking with pure pursuit
        self.global_goal_publisher = self.create_publisher(Odometry, 'global_goal_pure_pursuit', 1)
        self.offset_dist_publisher = self.create_publisher(Float32, 'standoff_dist_pure_pursuit', 1)
        self.local_goal_subscriber = self.create_subscription(Odometry, 'rrt_local_goal', self.local_goal_callback,1)

        self.use_rrt_subscriber = self.create_subscription(Bool, 'use_rrt', self.use_rrt_callback,1)

        self.rrt_spline_subscriber = self.create_subscription(PoseArray, 'rrt_spline_points', self.rrt_spline_callback,1)
        self.rrt_spline_publisher = self.create_publisher(Marker, 'rrt_spline_rviz',1)
        self.current_rrt_goal_publisher = self.create_publisher(PointStamped, 'rrt_current_goal_rviz', 1)

        self.use_rrt = False
        self.global_L = 1.5#2#5.0

        '''#Open csv and read the waypoint data
        with open(traj_csv, 'r') as f:
            lines = (line for line in f if not line.startswith('#'))
            data = np.loadtxt(lines, delimiter=';')
        self.goal_points = data[:, 1:3]/10
        x = self.goal_points[:, 0]
        y = self.goal_points[:, 1]

        spline_data, m = interpolate.splprep([x, y], s=0, per=True)
        self.x_spline, self.y_spline = interpolate.splev(np.linspace(0, 1, 1000), spline_data)
        self.spline_points = np.vstack((self.x_spline, self.y_spline, np.zeros((len(self.y_spline)))))
        self.xder, self.yder = interpolate.splev(np.linspace(0, 1, 1000), spline_data, der=2)
        self.derivatives1 = np.linalg.norm(np.expand_dims(self.xder, 1) + np.expand_dims(self.yder, 1), axis=1)  # dx
        self.derivatives2 = np.linalg.norm(np.expand_dims(self.xder, 1) - np.expand_dims(self.yder, 1), axis=1)  # dy
        '''
        rows = []
        with open("sim_points.csv", 'r') as file: 
            csvreader = csv.reader(file)
            header = next(csvreader)
            for row in csvreader:
                rows.append(row[0:2])
        self.goal_points=np.array(rows)
        self.spline_index = 99999
        x = self.goal_points[:,0]
        y = self.goal_points[:,1]
        x = np.append(x,x[0]).astype(float)
        y = np.append(y,y[0]).astype(float)
        spline_data, m = interpolate.splprep([x, y], s=0, per=True)
        self.x_spline, self.y_spline = interpolate.splev(np.linspace(0, 1, 1000), spline_data)
        self.xder, self.yder = interpolate.splev(np.linspace(0, 1, 1000), spline_data, der=2)
        self.derivatives1 = np.linalg.norm(np.expand_dims(self.xder,1) + np.expand_dims(self.yder,1),axis=1)#dx
        self.derivatives2 = np.linalg.norm(np.expand_dims(self.xder,1) - np.expand_dims(self.yder,1),axis=1)#dy
        
        self.spline_points=np.vstack((self.x_spline, self.y_spline, np.zeros((len(self.y_spline)))))

        self.rrt_spline_points=np.vstack((self.x_spline, self.y_spline, np.zeros((len(self.y_spline)))))

        self.spline_data = self.populate_spline_data()
        self.marker_data = self.populate_guide_points()

        #Publish Rviz Markers every 2 seconds
        self.timer = self.create_timer(2, self.create_spline)#Publish Spline
        self.timer = self.create_timer(2, self.publish_rviz_data)#Publish waypoints

        offset = 10 #Offset to make car brake before turn, and accelerate out of turn units in spline index stepss
        blur_value = 1 #Makes the transition from braking to full throttle longer (Can be any value 1 or larger)
        derivative_divisor = 600 #Changing this value will allow more or less sensitivity for turn angles
       
        threshold_value = 0.8
        #Find curvature of the spline
        self.derivatives = np.roll(np.where(((self.derivatives1 * self.derivatives2)/1000)/derivative_divisor > threshold_value, 1, 0),-700)
        self.derivatives  = np.convolve(self.derivatives,signal.gaussian(150, std=47), mode='same')
        self.derivatives = np.roll(self.derivatives,700)
        #print("max derivatives=",np.max(self.derivatives))
        #print("min derivatives=",np.min(self.derivatives))
        self.derivatives = np.where(self.derivatives / (np.max(self.derivatives) ) > 1, 1 ,self.derivatives / (np.max(self.derivatives)))# - 5) )
        #self.derivatives = self.derivatives/derivative_divisor
        
        #Offset the curvature by some value to provide early braking and early accelaration
        self.offset_points = np.roll(self.derivatives, - offset)

        self.local_goal = Odometry()

        self.time_since_last_point = self.get_clock().now()


    def rrt_spline_callback(self, points):
        all_points =[]
        for i in range(int(points.poses[0].position.z)):
            all_points.append([points.poses[i].position.x,points.poses[i].position.y])
        all_points = np.array(all_points).astype(float)

        if(points.poses[0].position.z > 3):
            x = all_points[:,0]
            y = all_points[:,1]
            spline_data, m = interpolate.splprep([x, y], s=0, per=True)
            self.rrt_x_spline, self.rrt_y_spline = interpolate.splev(np.linspace(0, 1, 25), spline_data)
        elif(points.poses[0].position.z == 3):
            x = all_points[:,0]
            y = all_points[:,1]
            path=[x,y]
            tck, u= interpolate.splprep(path, w=None, u=None, ub=None, ue=None, k=2, task=0, s=0.3, t=None, full_output=0, nest=None, per=0, quiet=1)
            xnew,ynew = interpolate.splev( np.linspace( 0, 1, 25), tck)
        else:
            x = all_points[:,0]
            y = all_points[:,1]
            y_flip=False
            if (x[0] > x[-1]):
                x=np.flip(x)
                y=np.flip(y)
                y_flip = True

            interp_range_x = np.linspace(min(x),max(x),25)
            self.rrt_y_spline = np.interp(interp_range_x, x, y)
            if y_flip == True:
                 self.rrt_y_spline = np.flip( self.rrt_y_spline)

            x_flip = False
            if (y[0] > y[-1]):
                x=np.flip(x)
                y=np.flip(y)
                x_flip = True
                
            interp_range_y = np.linspace(min(y),max(y),25)
            self.rrt_x_spline = np.interp(interp_range_y, y, x)
            if x_flip == True:
                self.rrt_x_spline = np.flip(self.rrt_x_spline)
        
        self.rrt_spline_points=np.vstack((self.rrt_x_spline, self.rrt_y_spline, np.zeros((len(self.rrt_y_spline)))))
        #Plot Spline on RVIZ
        spline_graph = self.rrt_populate_spline_data()

        marker = Marker()
        #marker.id = 0
        marker.action = Marker.DELETEALL
        self.rrt_spline_publisher.publish(marker)

        self.rrt_spline_publisher.publish(spline_graph)
 

    def use_rrt_callback(self, rrt_topic):
        self.use_rrt = rrt_topic.data
        self.time_since_last_point = self.get_clock().now()


    def local_goal_callback(self, local_goal_msg):
        #WARNING THIS VARIABLE IS MISNAMED, IT IS A GLOBAL POSE GOAL
        self.local_goal=local_goal_msg

    def global_2_local(self, current_quat, current_position, goal_point_global):
        # Construct transformation matrix from rotation matrix and position
        H_global2car = np.zeros([4, 4]) #rigid body transformation from  the global frame of referce to the car
        H_global2car[3, 3] = 1
        current_rotation_matrix = R.from_quat(np.array([current_quat.x,current_quat.y,current_quat.z,current_quat.w])).as_matrix()
        H_global2car[0:3, 0:3] = np.array(current_rotation_matrix)
        H_global2car[0:3, 3] = np.array([current_position.x, current_position.y, current_position.z])

        # Calculate point
        goal_point_global = np.append(goal_point_global, 1).reshape(4, 1)
        goal_point_car = np.linalg.inv(H_global2car) @ goal_point_global

        return goal_point_car

    def populate_guide_points(self):
        array_values=MarkerArray()

        for i in range(len(self.goal_points)):
            message = Marker()
            message.header.frame_id="map"
            message.header.stamp = self.get_clock().now().to_msg()
            message.type= Marker.SPHERE
            message.action = Marker.ADD
            message.id=i
            message.pose.orientation.x=0.0
            message.pose.orientation.y=0.0
            message.pose.orientation.z=0.0
            message.pose.orientation.w=1.0
            message.scale.x=0.25
            message.scale.y=0.25
            message.scale.z=0.25
            message.color.a=1.0
            message.color.r=1.0
            message.color.b=0.0
            message.color.g=0.0
            message.pose.position.x=float(self.goal_points[i,0])
            message.pose.position.y=float(self.goal_points[i,1])
            message.pose.position.z=0.0
            array_values.markers.append(message)
        return array_values
    def rrt_populate_spline_data(self):
        message = Marker()
        message.header.frame_id="map"
        message.type= Marker.LINE_STRIP
        message.action = Marker.ADD
        message.scale.x= 0.150
        message.pose.position.x= 0.0
        message.pose.position.y= 0.0
        message.pose.position.z=0.0
        message.color.a=1.0
        message.color.r=0.0
        message.color.b=1.0
        message.color.g=1.0
        message.pose.orientation.x=0.0
        message.pose.orientation.y=0.0
        message.pose.orientation.z=0.0
        message.pose.orientation.w=1.0

        for i in range(len(self.rrt_x_spline)-1):
            message.header.stamp = self.get_clock().now().to_msg()
            message.id=i
            point1=Point()
            point1.x=float(self.rrt_x_spline[i])
            point1.y=float(self.rrt_y_spline[i])
            point1.z=0.0

            message.points.append(point1)
            point2=Point()
            point2.x=float(self.rrt_x_spline[i+1])
            point2.y=float(self.rrt_y_spline[i+1])
            point2.z=0.0

            message.points.append(point2)
            self.rrt_spline_publisher.publish(message)

        message.id=len(self.rrt_x_spline)
        message.header.stamp = self.get_clock().now().to_msg()
        point1=Point()
        point1.x=float(self.rrt_x_spline[-1])
        point1.y=float(self.rrt_y_spline[-1])
        point1.z=0.0

        message.points.append(point1)

        point2=Point()
        point2.x=float(self.rrt_x_spline[0])
        point2.y=float(self.rrt_y_spline[0])
        point2.z=0.0

        message.points.append(point2)

        return message

    def populate_spline_data(self):
        message = Marker()
        message.header.frame_id="map"
        message.type= Marker.LINE_STRIP
        message.action = Marker.ADD
        message.scale.x= 0.150
        message.pose.position.x= 0.0
        message.pose.position.y= 0.0
        message.pose.position.z=0.0
        message.color.a=1.0
        message.color.r=1.0
        message.color.b=1.0
        message.color.g=1.0
        message.pose.orientation.x=0.0
        message.pose.orientation.y=0.0
        message.pose.orientation.z=0.0
        message.pose.orientation.w=1.0

        for i in range(len(self.x_spline)-1):
            message.header.stamp = self.get_clock().now().to_msg()
            message.id=i
            point1=Point()
            point1.x=float(self.x_spline[i])
            point1.y=float(self.y_spline[i])
            point1.z=0.0

            message.points.append(point1)
            point2=Point()
            point2.x=float(self.x_spline[i+1])
            point2.y=float(self.y_spline[i+1])
            point2.z=0.0

            message.points.append(point2)
            self.spline_publisher.publish(message)

        message.id=len(self.x_spline)
        message.header.stamp = self.get_clock().now().to_msg()
        point1=Point()
        point1.x=float(self.x_spline[-1])
        point1.y=float(self.y_spline[-1])
        point1.z=0.0

        message.points.append(point1)

        point2=Point()
        point2.x=float(self.x_spline[0])
        point2.y=float(self.y_spline[0])
        point2.z=0.0

        message.points.append(point2)

        return message

    def get_point_distances(self,points1, points2):
        return np.linalg.norm(points1-points2)

    def get_closest_point_to_car(self,current_position, all_points):
        try:
            current_position=np.array([current_position.x, current_position.y, current_position.z])
        except:
            current_position=np.array([current_position[0], current_position[1], current_position[2]])
        current_position=np.transpose(np.multiply(current_position,np.transpose(np.ones((all_points.shape)))))

        dist = np.linalg.norm(current_position - all_points, axis=0)

        point_index = np.argmin(dist)
        return point_index

    def get_path_distances(self,current_pos, points, current_point):

        pose=Point()
        pose.x=current_pos[0]
        pose.y=current_pos[1]
        pose.z=current_pos[2]
        closest_index = self.get_closest_point_to_car(pose, points)

        if current_point - closest_index > 4:
            a_vect=np.zeros((3,current_point - closest_index+1))
            b_vect=np.zeros((3,current_point - closest_index+1))

            a_vect[:,0]= current_pos
            a_vect[:,1:] = points[:,closest_index:current_point]

            b_vect = points[:,closest_index:current_point+1]

            distance=np.sum(np.linalg.norm(a_vect-b_vect, axis=0))
        else:
            distance = self.get_point_distances(current_pos, points[:,current_point])
        return distance

    def check_for_next_point(self,current_position, current_point_index, all_points, L):
        current_position=np.array([current_position.x, current_position.y, current_position.z])

        #Distance from current position to goal point
        #dist_to_goal = self.get_point_distances(current_position, all_points[:,current_point_index])

        #Distance Along Drive Path
        dist_to_goal = self.get_path_distances(current_position, all_points, current_point_index)

        if(dist_to_goal < L):
            current_point_index+=1
            if current_point_index >= len(all_points[0,:]):
                current_point_index=0
            self.publish_current_goal_point(all_points[:,current_point_index])
        elif(dist_to_goal > 4 * L):
            current_point_index = self.get_closest_point_to_car(current_position, all_points)
            self.publish_current_goal_point(all_points[:,current_point_index])

        return current_point_index

    def publish_rviz_data(self):
        self.goal_points_publisher.publish(self.marker_data)

    def publish_rrt_current_goal_point(self, goal_point_position):
        message=PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id="map"
        message.point.x=float(goal_point_position[0])
        message.point.y=float(goal_point_position[1])
        message.point.z=0.0
        self.current_rrt_goal_publisher.publish(message)

    def publish_current_goal_point(self, goal_point_position):

        message=PointStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id="map"
        message.point.x=float(goal_point_position[0])
        message.point.y=float(goal_point_position[1])
        message.point.z=0.0
        self.current_goal_publisher.publish(message)

    def create_spline(self):
        self.spline_publisher.publish(self.spline_data)

    def pose_callback(self, pose_msg):

        #parse information from pose_msg
        current_position = pose_msg.pose.pose.position
        current_quat = pose_msg.pose.pose.orientation

        # TODO: find the current waypoint to track using methods mentioned in lecture

        #parse information from pose_msg
        current_position = pose_msg.pose.pose.position
        current_quat = pose_msg.pose.pose.orientation

        L= self.global_L #2.5#3 * self.past_speed /4.5 #Look ahead distance

        # TODO: find the current waypoint to track using methods mentioned in lecture
        self.spline_index_car = self.get_closest_point_to_car(current_position, self.spline_points)

        global_goal_point = self.find_goal_point(L)
        
        timediff = self.get_clock().now() -  self.time_since_last_point

        #USE RRT if use_rrt is set to true
        if(self.use_rrt == True and timediff.nanoseconds * (1/1e9) < 0.2):
            self.rrt_spline_index_car = self.get_closest_point_to_car(current_position, self.rrt_spline_points)
            global_goal_point_rrt = self.rrt_find_goal_point(L/2)
            goal_point_car = self.global_2_local(current_quat, current_position, global_goal_point_rrt)
            self.publish_rrt_current_goal_point(global_goal_point_rrt)
        else:
            goal_point_car = self.global_2_local(current_quat, current_position, global_goal_point)
        
        #print(f"Position global: {current_position}")
        #print(f"Goal point global: {desired_point}")
        #print(f"Goal point car: {goal_point_car}")

        # USE THIS FOR K ON REAL CAR k = .15 and set drive speed to 10m/s
        kp=1#0.55

        y = goal_point_car[1]
        steer_dir = np.sign(y)
        r = L**2 / (2 * np.abs(y))
        gamma = 1/r

        steering_angle = (gamma * kp * steer_dir) # + (kd * kd_output)#How does gamma relate to the steering angle? -Lenny

        drive_speed = 0.5#8.0 # 6.5 #Change this to 10, when running on the real car
        min_linear_interp=0.01 #Curve value to activate braking
        max_linear_interp=1 #Max range of values for linear interpolation
        min_drive_speed = 3.0 #Min Speed on Turns

        #Turn back on below for the real car
        '''
        if self.offset_points[self.spline_index_car] > min_linear_interp:
            drive_speed = drive_speed - (((self.offset_points[self.spline_index_car] - min_linear_interp) / (max_linear_interp - min_linear_interp) ) * drive_speed)

            #Dont let the car drive too slow
            if drive_speed < min_drive_speed:
                drive_speed = min_drive_speed

            if drive_speed > 3.5:
                L_slow = 2
                k2 = 0.25
            else:
              L_slow = 1.0
              k2 = 0.25
            #Recalculate Steering Angle Here with a higher gain to account for needed corrections at a slower speed, perhaps try some type of linear interpolation?
            #L_slow=1.0
            global_goal_point = self.find_goal_point(L)
        
            #USE RRT if use_rrt is set to true
            if(self.use_rrt == True):
                goal_point_car =  np.array([self.local_goal.pose.pose.position.x, self.local_goal.pose.pose.position.y, 0])
            else:
                goal_point_car = self.global_2_local(current_quat, current_position, global_goal_point)
            
            #k2 = 0.25 #MAKE SURE THIS IS HIGHER THAN K1
            y = goal_point_car[1]
            steer_dir = np.sign(y)
            r = L_slow**2/(2*np.abs(y))
            gamma = 1/r
            steering_angle = gamma * k2 * steer_dir
        '''
        self.publish_current_goal_point(global_goal_point)#Create RVIZ Purple Dot
        # TODO: publish drive message, don't forget to limit the steering angle.
        msg = AckermannDriveStamped()
        msg.drive.speed = 1.0 #float(drive_speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_publisher.publish(msg)


        
        #Global Goal for RRT
        global_goal = Odometry()
        global_goal.header.frame_id="map"
        global_goal.header.stamp = self.get_clock().now().to_msg()
        global_goal.pose.pose.position.x = float(global_goal_point[0])
        global_goal.pose.pose.position.y = float(global_goal_point[1])
        global_goal.pose.pose.position.z = float(global_goal_point[2])
        self.global_goal_publisher.publish(global_goal)
    
    def rrt_find_goal_point(self, L):
        #Returns the global x,y,z position of the goal point
        points_in_front = np.roll(self.rrt_spline_points, -self.rrt_spline_index_car, axis=1)
        points_dist = np.linalg.norm(np.roll(points_in_front, 1, axis=1) - points_in_front, axis=0)
        points_dist = np.cumsum(points_dist)
        idx = np.argmin(np.abs(points_dist - L))
        goal_point_car = points_in_front[:, idx]
        return goal_point_car

    def find_goal_point(self, L):
        #Returns the global x,y,z position of the goal point
        points_in_front = np.roll(self.spline_points, -self.spline_index_car, axis=1)
        points_dist = np.linalg.norm(np.roll(points_in_front, 1, axis=1) - points_in_front, axis=0)
        points_dist = np.cumsum(points_dist)
        idx = np.argmin(np.abs(points_dist - L))
        goal_point_car = points_in_front[:, idx]

        return goal_point_car



def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()