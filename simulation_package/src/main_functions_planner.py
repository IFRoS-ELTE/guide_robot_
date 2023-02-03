import numpy as np
import math
from utils_lib.rrt_dubins_final import RRTDubins
from ompl import base as ob
from ompl import geometric as og

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data   # Map that have the free and occupied cell
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        p=self.__position_to_map__(pose) 
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribude). 
        # Be aware to only generate elements inside the map.    
        dis=int(self.distance/(self.resolution))
        if len(p) == 2: # if p is outside the map return true (unexplored positions are considered free)
            u_min = np.clip(p[0]-dis, 0,self.map.shape[0])
            v_min = np.clip(p[1]-dis, 0,self.map.shape[1])
            u_max = np.clip(p[0]+dis+1, 0,self.map.shape[0])
            v_max = np.clip(p[1]+dis+1, 0,self.map.shape[1])

            if np.any(self.map[u_min:u_max, v_min:v_max] > 0):
                return False     # Obstacle
        return True

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path, step_size=0.1):
        for i in range(len(path) - 1):
            # TODO: get dist and angle between current element and next element in path
            angle = math.atan2((path[i+1][1])-path[i][1],(path[i+1][0])-path[i][0])
            dist = math.sqrt((path[i][0] - path[i+1][0])**2 + (path[i][1] - path[i+1][1])**2)
            # dist=np.linalg.norm(path[i],path[i+1])
            for d in np.arange(0, dist, step_size):
                # TODO: check occupancy of each element d. If one element is occupied return False. \
                p = [path[i][0]+(d*np.cos(angle)),path[i][1]+(d*np.sin(angle))]
                if self.is_valid(p) ==False :
                    print('false')
                    return False
        return True

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):
        # TODO: convert world position to map coordinates
        uv = [int((p[0]-self.origin[0])/self.resolution),int((p[1]-self.origin[1])/self.resolution)]

        # keep position inside map
        if uv[0] < 0 or uv[0] >= self.map.shape[0] or uv[1] < 0 or uv[1] >= self.map.shape[1]:
            return []
        return uv
    
    def __map_to_position__(self,p):
        uv = [p[0,0]*self.resolution+self.origin[0],p[0,1]*self.resolution+self.origin[1]]
        return uv
    
    def __map_to_position_list__(self,p):
        uv = [p[0]*self.resolution+self.origin[0],p[1]*self.resolution+self.origin[1]]
        return uv
    

# # Planner: This function has to plan a path from start_p to goal_p by using the RRT Dubins
def compute_path( start_p, goal_p,k, step_size,turning_radius,delta_q, p,svc):
    qstart=svc.__position_to_map__(start_p)
    qstart.append(start_p[2])
    qgoal=svc.__position_to_map__(goal_p)
    yaw = np.random.uniform(-math.pi, math.pi,size=(1,1))
    qgoal.append(yaw)
    
    C=svc.map
    rrt_dubins=RRTDubins(qstart, qgoal,C, p,k, turning_radius, step_size,delta_q)
    path = rrt_dubins.planning()
    ret = []
    if path:   
        # If solved fill ret with the points [x, y] in the solution path
        for i in path:
            ret.append(svc.__map_to_position_list__(i))
    else:
        ret.append(start_p)
        print("No solution found")
    return ret


# # Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# # StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# # The planner returns a path that is a list of poses ([x, y]).
# def compute_path(start_p, goal_p, state_validity_checker, dominion, max_time=0.5):

#     # TODO: Plan a path from start_p to goal_p inside dominion using the OMPL and the state_validity_checker object. Follow notebook example.
#     # some code
    
#     # create an SE2 state space
#     space = ob.RealVectorStateSpace(2)

#     # Set the bounds of space to be in low to high.
#     space.setBounds(dominion[0], dominion[1])
    
#     # construct an instance of space information from this state space
#     si = ob.SpaceInformation(space)
    
#     # set state validity checking for this space
#     si.setStateValidityChecker(ob.StateValidityCheckerFn(state_validity_checker.is_valid))

#     # create start state
#     start = ob.State(space)
#     start[0] = start_p[0]
#     start[1] = start_p[1]
    
#     # create a goal state
#     goal = ob.State(space)
#     goal[0] = goal_p[0]
#     goal[1] = goal_p[1]

#     # create a problem instance
#     pdef = ob.ProblemDefinition(si)
    
#     # set the start and goal states
#     pdef.setStartAndGoalStates(start, goal)
    
#     # Create the optimization objective. Here it is to optimize the path lenght
#     pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(si))

#     # Construct the optimal planner. An RRT* planner is used.
#     optimizingPlanner = og.RRTstar(si)
#     # optimizingPlanner = og.RRTConnect(si)
#     # optimizingPlanner = og.RRT(si)
#     # optimizingPlanner = og.PRM(si)
#     # Set the problem instance for our planner to solve and call setup
#     optimizingPlanner.setProblemDefinition(pdef)
#     optimizingPlanner.setup()

#     # attempt to solve the planning problem in the given runtime
#     solved = optimizingPlanner.solve(max_time)
    
#     # Get planner data
#     pd = ob.PlannerData(si)
#     optimizingPlanner.getPlannerData(pd)

#     if solved:
#         # get the path and transform it to a list
#         path = pdef.getSolutionPath()
#         ret = []
#         # TODO: if solved fill ret with the points [x, y] in the solution path
#         for i in path.getStates():
#             ret.append(np.array([i[0], i[1]]))
#         if ret[-1][0]!=goal_p[0] and ret[-1][1]!=goal_p[1]:
#             ret.append(goal_p)
#     else:
#         print("No solution found")
#     return ret



# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(state, goal, Kv=0.8, Kw=1):
    # TODO: Implement a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # Make it sequential to avoid strange curves. First correct orientation and then distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    theta_d =  np.arctan2((goal[1]-state[1]), (goal[0]-state[0]))
    w = Kw* wrap_angle(theta_d - state[2])    # angular velocity
    if abs(theta_d-state[2])<0.08:
        v = Kv*math.sqrt((goal[0]-state[0])**2 + (goal[1]-state[1])**2)   # linear velocity
    else:
        v=0
    return v, w