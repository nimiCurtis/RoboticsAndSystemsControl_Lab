from multiprocessing import Process, Queue
from matplotlib.patches import Rectangle  , Circle, Arrow, ConnectionPatch  
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import numpy as np
from sympy import rotations

#from AStar import RRTStar, show_animation
from rrt_star import  RRTStar , show_animation

class Robot:
    def __init__(self,robot_id, A_r, disk, field):
        self.first_attack = True
        self.vel = 0.3 # m/s
        self.Rr = 0.06 # meters
        self.field = field
        self.disk = disk
        self.disk_zone = 1.05*(self.Rr + self.disk.Rd)
        self.robot_id = robot_id
        if (self.robot_id == 1) :
            self.right_corner = self.field.corners["RD"]
            self.left_corner = self.field.corners["RU"]

        elif(self.robot_id == 2) :
            self.right_corner = self.field.corners["LU"]
            self.left_corner = self.field.corners["LD"]

        self.initial_pose = A_r
        self.A_r = self.initial_pose.copy()
        self.p_r = self.A_r[:2 , 3]
        
        # self.A_o = A_o
        # self.p_e = self.A_o[:2, 3]
        self.res = 30

        self.total_path = np.empty((0,2))

        self.goal = False

    def set_plotter(self,plotter):
        self.plotter = plotter

    def set_pose(self, pose):
        self.A_r = pose
        self.p_r = self.A_r[:2 , 3]
    
    def reset_pose(self):
        self.A_r = self.initial_pose.copy()
        self.p_r = self.A_r[:2 , 3]
        self.total_path = np.empty((0,2))

    def get_pose(self):
        return self.A_r

    def get_position(self):
        return self.p_r

    def with_disk(self, disk_relative_to_robot ):
        distance_to_disk = np.linalg.norm(disk_relative_to_robot)
        if( distance_to_disk <= self.disk_zone) :
            return True
        else:
            return False

    def steering_angle(self, p_i):
        """
        Args:
            A r - Pose of its own robot relate to the base frame
            p i - next point in path relate to the base frame

        Returns:
            p_i_robot: 2d vector of next point in path with respect to robot frame: (x, y)
            phi: Steering angle to next point [degrees].
        """
        p_i = np.concatenate((p_i, [0,1]), axis=0) 
        p_relative_to_robot = np.matmul(np.linalg.inv(self.A_r), p_i)[0:2]
        phi = np.rad2deg(np.arctan2(p_relative_to_robot[1], p_relative_to_robot[0]))
        return p_relative_to_robot , phi

    def disk_pos_relative_to_robot(self):
        return self.steering_angle(self.disk.get_position())

    def set_opponent(self,robot):
        self.opponent_robot = robot

    def opponent_pos_relative_to_robot(self):
        return self.steering_angle(self.opponent_robot.get_position())

    def in_goal_zone(self):
        if (self.robot_id==1) and ((self.disk.p_d[0]+self.disk.Rd)>=self.field.x_range[1]):
            return True
        elif (self.robot_id==2) and ((self.disk.p_d[0]-self.disk.Rd)<=self.field.x_range[0]):
            return True
        else: 
            return False

    def linear_path(self , target_position):
        path = np.linspace(self.p_r, target_position, self.res)
        return path

    def corner_path(self):
        opponent_relative_to_robot, phi_to_opponent = self.opponent_pos_relative_to_robot()
        left_corner_relative_to_robot, phi_left_relative_to_robot= self.steering_angle(self.left_corner)
        right_corner_relative_to_robot, phi_right_relative_to_robot= self.steering_angle(self.right_corner)
        d_left=np.cross(left_corner_relative_to_robot, opponent_relative_to_robot)/np.linalg.norm(left_corner_relative_to_robot)
        d_right=np.cross(right_corner_relative_to_robot, opponent_relative_to_robot)/np.linalg.norm(right_corner_relative_to_robot)
        
        if np.abs(d_left) <= np.abs(d_right):
            path = self.linear_path(self.right_corner)
        else:
            path = self.linear_path(self.left_corner)
        return path ## may be path[::-1]
    
    def find_attacking_position(self):
        p_d = self.disk.get_position()
        return [p_d[0] - self.disk.Rd  , p_d[1]]

    def planner_path(self, O=[], B=[0,0.2 ], expand_dis = .1, path_resolution = 0.001, show_animation=True):
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
            robot_radius = self.Rr)

        path = rrt_star.planning(animation = show_animation)

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

        return path[::-1]


    def reach_disk(self, path):# q_r , q_e): # can't realy simulate this as it shuld be because we don't have input of the robot&disk positions from the camera
        pose = np.zeros((4,4)) 
        for next_pos in path:
            self.plotter.fig.clf()
            self.plotter.ax = self.plotter.fig.gca()
            self.plotter.plot()
            self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0)
            #self.ax.cla()
            #self.ax.set_xlim([-1,1])
            #self.ax.set_ylim([-0.5,0.5])
            p_relative_to_robot , phi = self.steering_angle(next_pos) # in the real assingment from this line we shoulded send the outputs to the motor commands 
            distance = np.linalg.norm(p_relative_to_robot)
            dt = distance/self.vel
            if dt==0: dt=0.0001
            plt.pause(dt)
            r = Rot.from_euler('z' , phi , degrees=True)
            rotation = np.matmul(r.as_matrix(),self.A_r[:3,:3]) # in simulation the robot turning in the exact amount of degrees
            position = np.concatenate([next_pos,[0,1]]) # the position will be exactly the next_pos 
            pose[:3,:3] = rotation
            pose[:,3] = position
            self.set_pose(pose) # updating robot pose
            #q_r.put(self.get_pose())
            #self.opponent_robot.set_pose(q_e.get())
            #self.ax.plot(position[0],position[1],markersize = 2.5, marker="o", color="b" )
            
            if (self.disk.is_moving()): # check every step that the disk is not moving, assuming that if the disk is moving is due to the enemy robot 
                return False
            else:
                disk_relative_to_robot , phi_to_disk = self.disk_pos_relative_to_robot()
                if self.with_disk(disk_relative_to_robot):
                    return True
                else:
                    continue
        self.plotter.fig.clf()
        self.plotter.ax = self.plotter.fig.gca()
        self.plotter.plot()
        self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0)

    def move_to_score(self, path):
        pose = np.zeros((4,4)) 
        #point = self.ax.plot(self.p_r[0],self.p_r[1],markersize = 6, marker="o", color="k" )
        for next_pos in path:
            self.plotter.fig.clf()
            self.plotter.ax = self.plotter.fig.gca()
            self.plotter.plot()
            self.total_path = np.append(self.total_path,np.array(next_pos).reshape(1,-1),axis=0)

            p_relative_to_robot , phi = self.steering_angle(next_pos) # in the real assingment from this line we shoulded send the outputs to the motor commands 
            distance = np.linalg.norm(p_relative_to_robot)
            dt = distance/self.vel
            if dt==0: dt=0.0001
            plt.pause(dt)
            r = Rot.from_euler('z' , phi , degrees=True)
            rotation = np.matmul(r.as_matrix(),self.A_r[:3,:3]) # in simulation the robot turning in the exact amount of degrees
            position = np.concatenate([next_pos,[0,1]]) # the position will be exactly the next_pos 
            pose[:3,:3] = rotation
            pose[:,3] = position
            self.set_pose(pose) # updating robot pose 
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

    def play(self, run_game ):#q_r , q_e):
        while(run_game):
            disk_relative_to_robot, phi_to_disk = self.disk_pos_relative_to_robot()
            if self.with_disk(disk_relative_to_robot):
                p_relative_to_robot , phi = self.steering_angle(self.disk.get_position()) # in the real assingment from this line we shoulded send the outputs to the motor commands 
                r = Rot.from_euler('z' , phi , degrees=True)
                rotation = np.matmul(r.as_matrix(),self.A_r[:3,:3]) # in simulation the robot turning in the exact amount of degrees 
                self.A_r[:3,:3] = rotation.copy()
                path = self.corner_path()
                if self.move_to_score(path):
                    self.goal = True
                    break
            else:
                    if self.disk.is_moving():
                        pass
                    else:
                        if self.first_attack:
                            path = self.linear_path(self.find_attacking_position())
                        else:
                            x_opponent = self.opponent_robot.get_position()[0]  
                            y_opponent = self.opponent_robot.get_position()[1]  
                            r_opponent =self.opponent_robot.Rr
                            obstacles = [(x_opponent,y_opponent,r_opponent)]
                            path = self.planner_path(O=obstacles)
                        self.reach_disk(path) # q_r , q_e)
                        continue


class Disk:
    def __init__(self, A_d):
        self.Rd = 0.035
        self.initial_pose = A_d
        self.A_d = self.initial_pose.copy()
        self.p_d = self.A_d[:2,3]
        self.last_p_d = self.p_d

    def set_position(self , pos):
        self.last_p_d = self.p_d.copy()
        self.p_d = pos
        self.A_d[:2,3] = pos

    def get_position(self):
        return self.p_d
    
    def reset_pose(self):
        self.A_d = self.initial_pose.copy()
        self.p_d = self.A_d[:2,3]
        self.last_p_d = self.p_d

    def is_moving(self):
        if(np.linalg.norm([self.last_p_d,self.p_d]) < 0.05) :
            return False
        else:
            return True
    
    def set_ax(self,ax):
        self.ax = ax

        


class Field:
    def __init__(self,length , width):
        self.length = length
        self.width = width
        self.x_range = [-length/2 , length/2]
        self.y_range = [-width/2 , width/2]
        self.corners = {"LU" : [self.x_range[0] , self.y_range[1]] , "LD" : [self.x_range[0] , self.y_range[0]] ,"RU" : [self.x_range[1] , self.y_range[1]] ,"RD" : [self.x_range[1] , self.y_range[0]]}

class the_robot_games:
    def __init__(self, disk , field , robot1 , robot2 ) :
        self.disk = disk
        self.field = field
        self.robot1 = robot1
        self.robot2 = robot2
        self.run_game = True    
        self.score = [0,0]


    def is_goal(self):
        if self.robot1.goal==True:
            self.score[0]+=1
            self.robot1.goal == False
        elif self.robot2.goal==True:
            self.score[1]+=1
            self.robot2.goal == False
        else:
            pass

    def reset_game(self):
        self.disk.reset_pose()
        self.robot1.reset_pose()
        

    def set_score(self):
        pass
    
    def get_score(self):
        pass
    

    

    def play_game(self):
        ## sequence 1
        # self.robot1.play(self.run_game)
        # q1 = Queue()
        # q2 = Queue()

        # p1 = Process(target=self.robot1.play , args=(self.run_game,q1,q2))
        # p2 = Process(target=self.robot2.play, args=(self.run_game,q2,q1))


        ## sequence 2
        self.reset_game()
        r = Rot.from_euler('z' , 15 , degrees=True)
        pose = np.zeros((4,4))
        rotation = np.matmul(r.as_matrix(),self.robot1.A_r[:3,:3]) # in simulation the robot turning in the exact amount of degrees 
        position = [-0.4, -0.32]
        pose[:3,:3] = rotation.copy()
        pose[:,3] = np.concatenate([position,[0 , 1]], axis=0)
        self.robot1.set_pose(pose)
        self.robot1.first_attack = False
        
        # p1.start()
        # p2.start()

        # p1.join()
        # p2.join()
        self.robot1.play(self.run_game)


        plt.show()
        self.is_goal() 
        #self.run_game = False
        
class Plotter:
    def __init__(self, disk , field , robot1 , robot2) :
        self.disk = disk
        self.field = field
        self.robots = [robot1 , robot2]
        self.fig , self.ax = plt.subplots()
    
    def plot_soccer_pitch(self):
        Pitch = Rectangle(self.field.corners["LD"], width = self.field.length, height = self.field.width , fill = False)
        midline = ConnectionPatch([0,-self.field.width/2],[0,self.field.width/2], "data" , "data")
        element = [Pitch , midline]
        for i in element:
            self.ax.add_patch(i)

    def plot_disk(self):
        disk = Circle(self.disk.get_position(),self.disk.Rd , color = 'k', alpha = 0.4)
        x_arrow = Arrow(*self.disk.get_position(), *self.disk.A_d[:2,0]*0.1, color='r', width = 0.03)
        y_arrow = Arrow(*self.disk.get_position(), *self.disk.A_d[:2,1]*0.1, color='g', width = 0.03)
        elements = [disk, x_arrow, y_arrow]
        for element in elements:
            self.ax.add_patch(element)
    
    def plot_robot(self):
        for robot in self.robots:
            robot_body = Circle(robot.get_position(),robot.Rr , color = 'b', alpha = 0.8)
            x_arrow = Arrow(*robot.get_position(), *robot.A_r[:2,0]*0.1, color='r', width = 0.03)
            y_arrow = Arrow(*robot.get_position(), *robot.A_r[:2,1]*0.1, color='g', width = 0.03)
            elements = [robot_body, x_arrow, y_arrow]
            for element in elements:
                self.ax.add_patch(element)

    def plot_path(self):
        for robot in self.robots:
            self.ax.plot(robot.total_path[:,0] ,robot.total_path[:,1] , '--p' , linewidth = 0.5 )

    def plot_settings(self):
        self.ax.set_title("THE ROBOT GAMES")
        self.ax.set_xlim([-1,1])
        self.ax.set_ylim([-0.5,0.5])

    def plot(self):
        self.plot_settings()
        self.plot_soccer_pitch()
        self.plot_disk()
        self.plot_robot()
        self.plot_path()

    def show(self):
        plt.show()


def init_field_status(field):
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




def main():
    field = Field(1.5,0.8)
    
    init_pose_disk , init_pose_robot1, init_pose_robot2 = init_field_status(field)
    
    disk = Disk(init_pose_disk)
    
    robot1 = Robot(1, init_pose_robot1, disk , field )
    robot2 = Robot(2, init_pose_robot2, disk , field )

    plotter = Plotter(disk , field , robot1, robot2)

    #plotter.plot()
    #plt.show()
    robot1.set_opponent(robot2)
    robot1.set_plotter(plotter)
    
    robot2.set_opponent(robot1)
    robot2.set_plotter(plotter)

    game = the_robot_games(disk , field, robot1, robot2)

    #p3 = Process(target=plotter.plot , args=(q1,q2,d,))

    game.play_game()

    a=1
    

if __name__ == "__main__":
    main()
