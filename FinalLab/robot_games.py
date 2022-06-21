from multiprocessing import Process, Queue
from matplotlib.patches import Rectangle  , Circle, Arrow, ConnectionPatch  
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from sympy import rotations


# dont think there is a different between the Astar file and the rrt_star
#from AStar import RRTStar, show_animation
from rrt_star import  RRTStar , show_animation

# define Robot object 
class Robot:
    def __init__(self,robot_id, A_r, disk, field):
        ## robot instances
        self.robot_id = robot_id # set robot id 1 / 2
        self.first_attack = True # determine whether it is the first possession of the game, default = true
        self.vel = 0.3 # robot velocity [m/s] for visualization
        self.radius = 0.06 # robot radius [meters] 
        
        # initialize pose 
        # A_r = homegeniouse matrix
        # p_r = posirion in (x,y) coordinate
        self.initial_pose = A_r
        self.A_r = self.initial_pose.copy()
        self.p_r = self.A_r[:2 , 3]
        
        ## other objects instances
        self.field = field # get field object
        self.disk = disk # get disk object
        self.disk_zone = 0.5*(self.radius + self.disk.radius) # tolerance from disk
        

        ## path instances
        self.res = 30 # linear path resolution
        self.total_path = np.empty((0,2))

        # set field corners depend on robot id
        if (self.robot_id == 1) : 
            self.right_corner = self.field.corners["RD"]
            self.left_corner = self.field.corners["RU"]

        elif(self.robot_id == 2) :
            self.right_corner = self.field.corners["LU"]
            self.left_corner = self.field.corners["LD"]

        # goal! = succeded to score
        self.goal = False

    def set_plotter(self,plotter):
        """
        Args:
            plotter: object of type Plotter 
        Returns:None
        """
        self.plotter = plotter

    def set_pose(self, pose):
        """
        set robot pose relate to world frame
        Args:
            pose: pose relate to world frame --> np.array(): shape=(4,4) 
        Returns: None
        """
        self.A_r = pose
        self.p_r = self.A_r[:2 , 3]

    def reset_pose(self):
        """
        initialize robot pose
        """
        self.A_r = self.initial_pose.copy()
        self.p_r = self.A_r[:2 , 3]
        self.total_path = np.empty((0,2))

    def get_pose(self):
        """
        get robot pose relate to world frame
        """
        return self.A_r

    def get_position(self):
        """
        get just the robot position relate to world frame
        """
        return self.p_r

    def transform(self,pose,phi,pos):
        """
        transform given pose depend on the z rotation angle and the translation
        Args: 
            pose: pose relate to world frame --> np.array(): shape=(4,4)
            phi: rotation angle in degrees --> int 
            pos: positin relate to world frame (x,y) coordinate --> np.array() or list
        Returns:
            pose: new pose after transformation 
        """
        r = Rot.from_euler('z' , phi , degrees=True) 
        rotation = np.matmul(r.as_matrix(),self.A_r[:3,:3]) # in simulation the robot turning in the exact amount of degrees
        position = np.concatenate([pos,[0,1]]) # the position will be exactly the next_pos after moving to the next point 
        pose[:3,:3] = rotation
        pose[:,3] = position
        return pose.copy()

    def with_disk(self, disk_relative_to_robot):
        """
        determine whether the robot holding the disk / located in the disk zone
        Args: 
            disk_relative_to_robot: position of the disk relate to the robot frame
        Returns:
            bool:   False - if the robot outside the disk zone
                    True - if the robot inside the disk zone
        """
        distance_to_disk = np.linalg.norm(disk_relative_to_robot)
        if( distance_to_disk <= self.disk_zone) :
            return True
        else:
            return False

    def steering_angle(self, p_i):
        """
        Args:
            p_i: next point in path relate to the base frame
        Returns:
            p_relative_to_robot: 2d vector of next point in path with respect to robot frame: (x, y)
            phi: Steering angle to next point [degrees].
        """
        p_i = np.concatenate((p_i, [0,1]), axis=0) 
        p_relative_to_robot = np.matmul(np.linalg.inv(self.A_r), p_i)[0:2]
        phi = np.rad2deg(np.arctan2(p_relative_to_robot[1], p_relative_to_robot[0]))
        return p_relative_to_robot , phi

    def disk_pos_relative_to_robot(self):
        """
        Returns:
            disk_relative_to_robot: 2d vector of disk with respect to robot frame: (x, y)
            phi: Steering angle to the disk [degrees].
        """
        return self.steering_angle(self.disk.get_position())

    def set_opponent(self,robot):
        """
        set the opponent robot
        """
        self.opponent_robot = robot

    def opponent_pos_relative_to_robot(self):
        """
        Returns:
            opponent_relative_to_robot: 2d vector of opponent with respect to robot frame: (x, y)
            phi: Steering angle to the opponent [degrees].
        """
        return self.steering_angle(self.opponent_robot.get_position())

    def in_goal_zone(self):
        """
        checking if disk passed the goal line depending on the robot id
        Returns:
            bool:   False - if the disk not passed the goal line
                    True - if disk is behind the goal line
        """
        if (self.robot_id==1) and ((self.disk.p_d[0]+self.disk.radius)>=self.field.x_range[1]):
            return True
        elif (self.robot_id==2) and ((self.disk.p_d[0]-self.disk.radius)<=self.field.x_range[0]):
            return True
        else: 
            return False

    def linear_path(self , target_position):
        """
        calculate linear path
        Args:
            target_position: position in (x,y) coordinates with respect to the world frame --> lisr ot np.array
        Returns:
            path: np.array of positions which located on the linear line to the target position depending on the resolution
        """
        path = np.linspace(self.p_r, target_position, self.res)
        return path

    def corner_path(self):
        """
        calculate linear path to the corners and selecting the corner path which the opponent has the longest way to reach it
        Returns:
            path: np.array of positions which located on the linear line to the corner
        """
        # calculate opponent norms to the corners linear paths  
        opponent_relative_to_robot, phi_to_opponent = self.opponent_pos_relative_to_robot() 
        left_corner_relative_to_robot, phi_left_relative_to_robot= self.steering_angle(self.left_corner)
        right_corner_relative_to_robot, phi_right_relative_to_robot= self.steering_angle(self.right_corner)
        d_left=np.cross(left_corner_relative_to_robot, opponent_relative_to_robot)/np.linalg.norm(left_corner_relative_to_robot)
        d_right=np.cross(right_corner_relative_to_robot, opponent_relative_to_robot)/np.linalg.norm(right_corner_relative_to_robot)
        
        if self.opponent_robot.get_position()[0]<self.disk.get_position()[0]: # check if the disk is behind the opponent or infront
            if np.abs(d_left) <= np.abs(d_right): # if disk is behind --> go the path with the shortest norm  
                path = self.linear_path(self.left_corner)
            else:
                path = self.linear_path(self.right_corner)
        else: # if disk is infront --> go the path with the longest norm
            if np.abs(d_left) <= np.abs(d_right):
                path = self.linear_path(self.right_corner)
            else:
                path = self.linear_path(self.left_corner)
        return path 
    
    def find_attacking_position(self):
        """
        calculate the position on the disk radius between the disk<>own goal line
        Returns:
            position on the disk radius in (x,y) coordinates --> list
        """
        p_d = self.disk.get_position()
        return [p_d[0] - self.disk.radius  , p_d[1]]

    def planner_path(self, O=[], B=[-0.5,0.5 ], expand_dis = .03, path_resolution = 0.001, show_animation=True):
        """
        Args:
            Pc: start point (x_s, y_s) --> list: len=2 OR np.array(): shape=(2,)
            Pg: end point (x_g, y_g) --> list: len=2 OR np.array(): shape=(2,)
            O: [(x_obs_1, y_obs_2, radius_obs_1), ..., (x_obs_N, y_obs_N, radius_obs_N)
            B: this is the area where you plan the path in. both x and y should be in between these boundaries.
            delta: Path resolution.
            **args: add additional arguments as you please such as whether to plot a path or not.
        Returns:
            path: [[x_1, y_1], [x_2, y_2], ..., [x_M, y_M]] --> List of lists of size 2 (x, y).
                    Important: Make sure that your output 'path' is in the right order (from start to goal)
        """
        rrt_star = RRTStar(
            start = self.get_position(),
            goal = self.find_attacking_position(),
            obstacle_list = O,
            rand_area = B,
            expand_dis = expand_dis,
            path_resolution = path_resolution,
            robot_radius = self.radius)
        path = rrt_star.planning(animation = show_animation)
        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")
        return path[::-1]

    def reach_disk(self, path):# q_r , q_e): # can't realy simulate this as it shuld be because we don't have input of the robot&disk positions from the camera
        """
        check if the robot reached the disk 
        Args:
            path: positions array in (x,y) coordinates with respect to the world frame --> list or np.array from planner or linear path 
        Returns:
            bool:   False - disk is moving because opponent robot is holding it
                    True - robot in disk zone
        """
        self.disk.set_position(self.disk.get_position())
        pose = np.zeros((4,4)) 
        for next_pos in path:
            self.plotter.plot() # plot all
            self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0) # appending the path positions
            p_relative_to_robot , phi = self.steering_angle(next_pos) # in the real assingment from this line we shoulded send the outputs to the motor commands 
            distance = np.linalg.norm(p_relative_to_robot) 
            dt = distance/self.vel # calculate dt[seconds] depend on the distance to next point and robot velocity
            if dt==0: dt=0.0001 
            plt.pause(dt) # "simulate" the plots with respect to robot velocity
            pose = self.transform(pose,phi,next_pos)
            self.set_pose(pose) # updating robot pose
            if (self.disk.is_moving()): # check every step that the disk is not moving, assuming that if the disk is moving is due to the enemy robot 
                return False
            else:
                disk_relative_to_robot , phi_to_disk = self.disk_pos_relative_to_robot()
                if self.with_disk(disk_relative_to_robot): # check if the robot reached the disk
                    return True
                else: # if not --> continue to next iteration
                    continue
        self.plotter.plot()
        self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0)

    def move_to_score(self, path):
        """
        check if the robot score
        Args:
            path: positions array in (x,y) coordinates with respect to the world frame --> list or np.array from planner or linear path 
        Returns:
            bool:   False - the robot lost the disk
                    True - disk in goal zone
        """
        pose = np.zeros((4,4)) 
        for next_pos in path:
            self.plotter.plot() # plot all
            self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0) # appending the path positions
            p_relative_to_robot , phi = self.steering_angle(next_pos) # in the real assingment from this line we shoulded send the outputs to the motor commands 
            distance = np.linalg.norm(p_relative_to_robot) 
            dt = distance/self.vel #calculate dt[seconds] depend on the distance to next point and robot velocity
            if dt==0: dt=0.0001
            plt.pause(dt)
            pose = self.transform(pose,phi,next_pos)
            self.set_pose(pose.copy()) # updating robot pose 
            self.disk.set_position(next_pos) # in simulation the disk is moving with the robot
            disk_relative_to_robot, phi_to_disk = self.disk_pos_relative_to_robot()
            if self.with_disk(disk_relative_to_robot):
                if self.in_goal_zone():
                    return True
                else:
                    continue
            else:
                return False
        self.plotter.fig.clf()
        self.plotter.ax = self.plotter.fig.gca()
        self.plotter.plot()
        self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0)

    def play(self, run_game ):
        """
        play until there is a goal
        Args:
            run_game(bool): determine wether the game is runing or not , default = True --> game is runing  
        """
        while(run_game): # while game is running
            disk_relative_to_robot, phi_to_disk = self.disk_pos_relative_to_robot()
            if self.with_disk(disk_relative_to_robot): # check if the robot holding the disk
                p_relative_to_robot , phi = self.steering_angle(self.disk.get_position()) # in the real assingment from this line we shoulded send the outputs to the motor commands 
                r = Rot.from_euler('z' , phi , degrees=True) 
                rotation = np.matmul(r.as_matrix(),self.A_r[:3,:3]) # in simulation the robot turning in the exact amount of degrees 
                self.A_r[:3,:3] = rotation.copy() # rotate to the disk 
                path = self.corner_path() # get path to the requierd corner
                if self.move_to_score(path): # if the robot score return True and stop the loop
                    self.goal = True
                    break
            else:
                    if self.disk.is_moving(): # if the disk is moving go in straight line to its position
                        path = self.linear_path(self.find_attacking_position())
                        self.reach_disk(path) # try to reach the disk
                        continue
                    else:
                        if self.first_attack: # if it is the first possesion of the game go in straight line to the disk = faster then the planner
                            path = self.linear_path(self.find_attacking_position())
                        else: # plan a path with respect to the opponent position --> good for situation when the opponent is in between the robot and the disk  (and the disk is not moving)
                            x_opponent = self.opponent_robot.get_position()[0]  
                            y_opponent = self.opponent_robot.get_position()[1]  
                            r_opponent =self.opponent_robot.radius
                            obstacles = [(x_opponent,y_opponent,r_opponent)]
                            path = self.planner_path(O=obstacles) 
                        self.reach_disk(path) # q_r , q_e)
                        continue

# define Disk object 
class Disk:
    def __init__(self, A_d):
        # disk instances
        self.radius = 0.035 

        # initialize pose and last position 
        self.initial_pose = A_d 
        self.A_d = self.initial_pose.copy()
        self.p_d = self.A_d[:2,3]
        self.last_p_d = self.p_d

    def set_position(self , pos):
        """
        set disk position relate to world frame
        Args:
            pos: position relate to world frame in (x,y) coordinates --> list or np.array 
        Returns: None
        """
        self.last_p_d = self.p_d.copy()
        self.p_d = pos
        self.A_d[:2,3] = pos

    def get_position(self):
            """
            get disk position (x,y) coordiantes --> np.array
            """
            return self.p_d

    def reset_pose(self):
        """
        reset disk pose to the initial pose 
        """
        self.A_d = self.initial_pose.copy()
        self.p_d = self.A_d[:2,3]
        self.last_p_d = self.p_d

    def is_moving(self):
        """
        Returns:
                bool:   False - if the disk is not moving AKA the error from last position is large then some tolerance
                        True - if the error is small 
        """
        epsilon = 0.05 # good for the real world, in simulation the movements is forced
        if(np.linalg.norm([self.last_p_d[0]-self.p_d[0],self.last_p_d[1]-self.p_d[1]]) < epsilon) :
            return False
        else:
            return True

# define Field object 
class Field:
    def __init__(self,length , width):
        # field variables
        self.length = length 
        self.width = width
        self.x_range = [-length/2 , length/2]
        self.y_range = [-width/2 , width/2]
        self.corners = {"LU" : [self.x_range[0] , self.y_range[1]] , "LD" : [self.x_range[0] , self.y_range[0]] ,"RU" : [self.x_range[1] , self.y_range[1]] ,"RD" : [self.x_range[1] , self.y_range[0]]}

# define the_robot_games object
class the_robot_games:
    def __init__(self, disk , field , robot1 , robot2 ) :
        # objects instances
        self.disk = disk
        self.field = field
        self.robot1 = robot1
        self.robot2 = robot2

        # game varibles
        self.run_game = True    
        self.score = [0,0]

    def is_goal(self):
        """
        check if ther was a goal and who did it for updating the score 
        Returns:
                bool:   False - there wasn't  a goal
                        True - there was a goal 
        """
        if self.robot1.goal==True:
            self.score[0]+=1
            self.robot1.goal == False
        elif self.robot2.goal==True:
            self.score[1]+=1
            self.robot2.goal == False
        else:
            pass

    def reset_game(self):
        """
        reset poses of the robots and disk
        """
        self.disk.reset_pose()
        self.robot1.reset_pose()
        self.robot2.reset_pose()

    def play_game(self):
        """
        play game 
        """
        ## sequence 1
        pose2 = np.zeros((4,4))
        position2 = [0.1, 0.1]
        pose2 = self.robot2.transform(pose2, 20 , position2)
        self.robot2.set_pose(pose2.copy())
        self.robot1.play(self.run_game)

        ## sequence 2
        self.reset_game()
        pose1 = np.zeros((4,4))
        position1 = [-0.65, -0.36]
        pose1 = self.robot1.transform(pose1,15 , position1)
        self.robot1.set_pose(pose1.copy())
        self.robot1.first_attack = False

        self.disk.set_position([-0.4,-0.23])
        self.disk.set_position([-0.4,-0.23]) # double to set that the disk is not moving
        
        pose2 = np.zeros((4,4))
        position2 = [0.4, - 0.3]
        pose2 = self.robot2.transform(pose2, 0 , position2)
        self.robot2.set_pose(pose2.copy())
        
        self.robot1.play(self.run_game)

        ## sequence 3
        self.reset_game()
        pose1 = np.zeros((4,4))
        position1 = [-0.65, -0.36]
        pose1 = self.robot1.transform(pose1,15 , position1)
        self.robot1.set_pose(pose1.copy())
        self.robot1.first_attack = False

        pose2 = np.zeros((4,4))
        position2 = [-0.3, -0.1]
        pose2 = self.robot2.transform(pose2, 0 , position2)
        self.robot2.set_pose(pose2.copy())
        
        self.robot1.play(self.run_game)

        # sequence 4
        self.reset_game()
        pose1 = np.zeros((4,4))
        position1 = [0.5,-0.1]
        pose1 = self.robot1.transform(pose1,15 , position1)
        self.robot1.set_pose(pose1.copy())
        self.robot1.first_attack = False
        
        self.disk.set_position([-0.22,0.23])

        pose2 = np.zeros((4,4))
        position2 = [-0.13, 0.26]
        pose2 = self.robot2.transform(pose2, 0 , position2)
        self.robot2.set_pose(pose2.copy())

        self.robot1.play(self.run_game)

        # sequence 5
        self.reset_game()
        pose1 = np.zeros((4,4))
        position1 = [-0.7,-0.1]
        pose1 = self.robot1.transform(pose1,15 , position1)
        self.robot1.set_pose(pose1.copy())
        self.robot1.first_attack = False
        
        self.disk.set_position([-0.1,-0.23])

        pose2 = np.zeros((4,4))
        position2 = [-0.04, -0.26]
        pose2 = self.robot2.transform(pose2, 0 , position2)
        self.robot2.set_pose(pose2.copy())

        self.robot1.play(self.run_game)

        plt.show()

# define Plotter object
class Plotter:
    def __init__(self, disk , field , robot1 , robot2) :
        # objects instances
        self.disk = disk
        self.field = field
        self.robots = [robot1 , robot2]
        
        # init fig an axes
        self.fig , self.ax = plt.subplots()

    def plot_soccer_pitch(self):
        """
        plot the field depend on its size
        """
        Pitch = Rectangle(self.field.corners["LD"], width = self.field.length, height = self.field.width , fill = False)
        midline = ConnectionPatch([0,-self.field.width/2],[0,self.field.width/2], "data" , "data")
        element = [Pitch , midline]
        for i in element:
            self.ax.add_patch(i)

    def plot_disk(self):
        """
        plot the disk and its coordinate system
        """
        disk = Circle(self.disk.get_position(),self.disk.radius , color = 'k', alpha = 0.8)
        x_arrow = Arrow(*self.disk.get_position(), *self.disk.A_d[:2,0]*0.1, color='r', width = 0.03)
        y_arrow = Arrow(*self.disk.get_position(), *self.disk.A_d[:2,1]*0.1, color='g', width = 0.03)
        elements = [disk, x_arrow, y_arrow]
        for element in elements:
            self.ax.add_patch(element)

    def plot_robot(self):
        """
        plot the robots and their coordinate system
        """
        for robot in self.robots:
            robot_body = Circle(robot.get_position(),robot.radius , color = 'b', alpha = 0.4)
            x_arrow = Arrow(*robot.get_position(), *robot.A_r[:2,0]*0.1, color='r', width = 0.03)
            y_arrow = Arrow(*robot.get_position(), *robot.A_r[:2,1]*0.1, color='g', width = 0.03)
            elements = [robot_body, x_arrow, y_arrow]
            for element in elements:
                self.ax.add_patch(element)

    def plot_path(self):
        """
        plot the path done by each robot
        """
        for robot in self.robots:
            self.ax.plot(robot.total_path[:,0] ,robot.total_path[:,1] , '--p' , linewidth = 0.5 )

    def plot_settings(self):
        """
        set general ax properties
        """
        self.ax.set_title("THE ROBOT GAMES")
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-0.5,0.5])

    def plot(self):
        """
        plot all
        """
        self.fig.clf() # clean frame and plot a new one
        self.ax = self.fig.gca() 
        self.plot_settings()
        self.plot_soccer_pitch()
        self.plot_disk()
        self.plot_robot()
        self.plot_path()


def init_field_status(field):
    """
        initialize the instances as would happen in the real world from the camera
        Arg:
            field: Field object
        Returns:
            init_pose_disk: pose of disk with respect to world/field frame
            init_pose_robot1: pose of robot1 with respect to world/field frame
            init_pose_robot2: pose of robot2 with respect to world/field frame
    """
    field = field

    init_pose_disk = np.eye(4)
    init_position_disk = [0,0]
    init_pose_disk[:2,3] = init_position_disk
    
    init_pose_robot1 = np.eye(4)
    init_position_robot1 = [-field.length/2,0]
    init_pose_robot1[:2,3] = init_position_robot1

    init_pose_robot2 = np.eye(4)
    init_pose_robot2[:2,:2]*=-1
    init_position_robot2 = [field.length/2,0]
    init_pose_robot2[:2,3] = init_position_robot2
    
    return init_pose_disk , init_pose_robot1, init_pose_robot2

## main program
def main():
    field = Field(1.5,0.8)
    # init status
    init_pose_disk , init_pose_robot1, init_pose_robot2 = init_field_status(field) 
    #init objects
    disk = Disk(init_pose_disk) 
    robot1 = Robot(1, init_pose_robot1, disk , field )
    robot2 = Robot(2, init_pose_robot2, disk , field )
    plotter = Plotter(disk , field , robot1, robot2)
    
    # set opponent and plotter
    robot1.set_opponent(robot2)
    robot1.set_plotter(plotter)
    robot2.set_opponent(robot1)
    robot2.set_plotter(plotter)

    # init game
    game = the_robot_games(disk , field, robot1, robot2)
    # play a game
    game.play_game()

if __name__ == "__main__":
    main()