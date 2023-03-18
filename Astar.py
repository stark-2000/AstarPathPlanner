import sys
import time

import heapq

from node import *
from node_manager import *
from environment import *
from action_handler import *



#Git repo link https://github.com/itej89/SimplePathPlanner

 
class astar_planner:

    def __init__(self, env, action_handler) -> None:
        self.env = env
        self.action_handler = action_handler
        
    def explore_actions(self, parent_node) -> bool:
        """Function takes an environmet state and runs 
        possible actions for that state and extract 
        possible unique future states and stores them in the que

        Args:
            parent_node (node): node representing a state of the
            environment

        """

        #get the position of the agent location
        # this represents the (x,y) position on the grid
        agents_postion = parent_node.Node_State

        #move the input node into explored list
        self.visited_node_list[agents_postion] = None

        Sim_Pose = None
        Status = False

        #iterate through all possible actions
        for action in self.env.Actions:
            #make a copy of the current agents position

            #Compute agents possible furture position
            Status ,Sim_Pose = self.action_handler.ActionHandlers[action](parent_node)

            #If the future position is in the bounds of the environemnt
            if Status:
                
                simulated_position , Total_Cost_To_Come = Sim_Pose

                #else of the state has already been explored then ignore the action
                if simulated_position in self.visited_node_list:
                    continue
                
                NewNode = None
                #if state has been visited for the first time then create anew node for the state
                if simulated_position not in node_manager.global_node_directory:
                    #Create a new node
                    NewNode = node_manager.make_node(simulated_position, Total_Cost_To_Come)
                    #Update the parent for the node
                    NewNode.Parent_Node_hash = parent_node.Node_hash
                
                
                #else if the state is in pending que then verify its current cost 
                # and update its cost and parent if necessary
                else:
                    #If node can be reached in less cost
                    if node_manager.global_node_directory[simulated_position].Cost_to_Come > Total_Cost_To_Come:
                        #update cost
                        node_manager.global_node_directory[simulated_position].Cost_to_Come = Total_Cost_To_Come
                        #update new parent node
                        node_manager.global_node_directory[simulated_position].Parent_Node_hash = parent_node.Node_hash
                        NewNode = node_manager.global_node_directory[simulated_position]

                #Push new node to the pending que for future expoloration
                if NewNode != None:
                    #Found an unique state that needs to be pushed in to the que
                    heapq.heappush(self.pending_state_que, NewNode)

    
    def find_goal_node(self, start_state, goal_state) -> bool:
        """Takes start stat and goal state for the environement 
        and computes the tree using _Breadth first search algorithm till the goal state is reached 

        Args:
            start_state (2D numpy array): Start state for the environment
            goal_state (2D numpy array): Goal state for the environment
        """

        #Call this fuction for auto id generation of nodes for a new tree
        node_manager.initialize()

        #initailize states
        self.goal_state = goal_state
        self.initial_state = start_state
        self.Final_Node = None

        #Initialize search que. This stores the nodes that needs to be explored
        start_node = node_manager.make_node(self.initial_state)
        self.pending_state_que = [start_node]
        heapq.heapify(self.pending_state_que)

        self.visited_node_list = {}

        #Perform search till a goal state is reached or maximum number of iterations reached
        
        while True:
            if len(self.pending_state_que) > 0:
                #fetch next node to be explored
                next_item = heapq.heappop(self.pending_state_que)
            else:
                next_item = None
                
            if next_item!= None:
                next_node = next_item

                #Check if the next node is the goal node, then return success
                if next_node.Node_State == self.goal_state:
                    self.Final_Node = next_node
                    print("Found the goal state!!!")
                    return True

                #Check if the next node has already been visited then ignore the node
                if next_node.Node_State in self.visited_node_list:
                    continue
                ##lse explore the node
                else:
                    self.explore_actions(next_node)
            else:
                print("Unable to find the goal state!!!")
                return False

    def back_track(self):
        """Funciton to find the final trajectory given the final goal node

        Returns:
            list of tuples: list of trajectory points
        """
        if self.Final_Node != None:
            last_node = self.Final_Node
            trajectory = [last_node.Node_State]

            #back track untill the start node has been reached
            while True:
                last_node = node_manager.global_node_directory[last_node.Parent_Node_hash]
                if last_node.Parent_Node_hash != None:
                    trajectory.append(last_node.Node_State)
                else:
                    break

            trajectory.reverse()
            return trajectory
        
        return None
    
    def visualize_exploration(self):
        """Visualize exploration of the nodes
        """
        #indicate start and goal nodes with circles
        self.env.highlight_state(start_state)
        self.env.highlight_state(goal_state)
        self.env.refresh_map()
        cv.waitKey(1)

        #variable to control gui update rate
        update_count = 0
        # Loop through all visited nodes and show on the cv window
        for position in self.visited_node_list:
            self.env.update_map(position)
            self.env.highlight_state(self.initial_state)
            self.env.highlight_state(self.goal_state)
            _environment.refresh_map()

            update_count +=1
            #Update gui every 300 iteration
            if update_count == 500:
                update_count = 0
                _environment.write_video_frame()
                cv.waitKey(1)



     

    def visualize_trajectory(self, trajectory):
        """Funciton to visualize trajectory

        Args:
            trajectory (lsit of tuples): trajectory
        """
        #Loop through all points in the trajectory
        for point in trajectory:
            _environment.highlight_point(point)
        #upodate map with the trajectory
        _environment.refresh_map()
        for i in range(20):
            _environment.write_video_frame()
        cv.waitKey(1)

if __name__ == "__main__":

    if len(sys.argv) == 3:

        # str_start_pos = sys.argv[1].replace("[", "").replace("]", "").replace(" ", "").split(",")
        # str_goal_pos = sys.argv[2].replace("[", "").replace("]", "").replace(" ", "").split(",")
        # #define goal and start state
        # start_state = (int(str_start_pos[1]), int(str_start_pos[0]))
        # goal_state  = (int(str_goal_pos[1]), int(str_goal_pos[0]))

        start_state = (30, 30, 30)
        goal_state = (40, 40, 30)

        #Create environment
        print("Please wait while creating the environment.")
        _environment = environment(250, 600)
        _environment.create_map()
        print("Environment created successfully.")

        good_states = True
        #Check if the given state are valid
        if not _environment.is_valid_position(start_state):
            print(f"Invalid start state.") 
            good_states = False
            
        if not _environment.is_valid_position(goal_state):
            print(f"Invalid goal state.") 
            good_states = False
            
        
        if good_states:
            #Create action handler for the environment
            _actionHandler = action_handler(_environment)

            #create planner
            _astar_planner = astar_planner(_environment, _actionHandler)

            #Request planner to find a plan
            print("Planning started.")
            print(f"Start Psotion : {[start_state[1], start_state[0]]}, Goal Postion : {[goal_state[1], goal_state[0]]}")
            print("Please wait while searching for the goal state...")
            start_time = time.time()
            _status =  _astar_planner.find_goal_node(start_state, goal_state)
            elspsed_time = time.time() - start_time

            print(f"Total time taken to run the algorithm : {elspsed_time} seconds\n")

            #If Planner is successfull, visualize the plan
            if _status:
                trajectory = _astar_planner.back_track()
                if trajectory != None:
                    _environment.begin_video_writer()
                    _astar_planner.visualize_exploration()
                    _astar_planner.visualize_trajectory(trajectory)
                    _environment.close_video_writer()

                    _environment.save_image("Solution.png")
                    cv.waitKey(0)

    else:
        print(f"Invalid number of command line arguments recieved! Please remove any unncessary spaces and try again! \n\n \
Please run the program as : 'python3 dijkstra_venkatatejkiranreddy_polamreddy.py  [Start_X,Start_Y] [Goal_X,Goal_Y]'\n")