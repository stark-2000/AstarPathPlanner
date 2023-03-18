import math
import numpy as np

class action_handler:
#Class to handle the action supported by the environment


    def __init__(self, env, step_size, start_pose, goal_pose) -> None:
        """Initilaize action handler parameters
        Args:
            env (environment): referance ot environment
        """
        self.env = env
        self.start_pose = start_pose
        self.goal_pose = goal_pose
        self.step_size = step_size

        #define al the actions possible in the environment
        self.Actions = {-60, -30, 0, 30, 60}

    
    def ActionValues(self, action,  pose):
        x, y, theta = pose

        theta_new = theta + pose
        theta_new_rad = theta_new*math.pi/180

        x_new = x + self.step_size * math.cos( action+theta_new_rad ) 
        y_new = y + self.step_size * math.sin( action+theta_new_rad )

        return (round(x_new), round(y_new), theta_new) 
    
    def cost_norm(self, pose1, pose2):
        x1, y1, theta1 = pose1
        x2, y2, theta2 = pose2
        np.linalg.norm([x1 - x2, y1 - y2])

    def PerformAction(self, parent_node, action):
        """Compute the next state and estimate the total cost for the next state

        Args:
            parent_node (node): node of the 
            action (string): action label

        Returns:
            tuple: validity of the action, action cost
        """

        agents_pose = parent_node.Node_State

        #Simulate agents nect position
        simulated_position = self.ActionValues(action, agents_pose)
        
        #Compute the total cost of the action
        action_cost_to_come = parent_node.Cost_to_Come + self.cost_norm(agents_pose, simulated_position)

        action_cost_to_go = parent_node.Cost_to_Come + self.cost_norm(simulated_position, self.goal_pose)

        #Check if the computed position os valid
        if not self.env.is_valid_position(simulated_position):
                return False, None
        
        #return the estimated position and cost
        return True, (simulated_position, action_cost_to_come, action_cost_to_go)

