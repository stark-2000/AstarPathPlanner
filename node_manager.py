
from node import *


class node_manager:
#Class to mange all the node creation and maintainance

    #A global node directory where all the nodes that has been creates will be stored
    global_node_directory = {}

    node_x = []
    node_y = []
    node_theta = []


    #call this funciton before starting the search to reset id
    def initialize():
        node_manager.global_node_directory.clear()


    #Funciton to create a hash map for a given state of the node
    def make_hash(state):
        #For the current use case we can just use the lcoation of the pixel as hash
        node_manager.node_x.append(state[0])
        node_manager.node_y.append(state[1])
        node_manager.node_theta.append(state[2])
        return len(node_manager) - 1


    def make_node(state, cost_to_come = 0, cost_to_go = 0):
        """Funciton creates a Node using the given parameterts

        Args:
            state (tuple): location of the node
            cost_to_come (int, optional): cost to come to the node. Defaults to 0.
            cost_to_go (int, optional): cost to go from the node. Defaults to 0.

        Returns:
            node: referance to the new node stored the global directory
        """
        #Make hash for the node
        hash = node_manager.make_hash(state)
        
        #Create a node and store it in global node directory
        node_manager.global_node_directory[hash] = node(state, hash, cost_to_come, cost_to_go)

        #Return referance to the new node
        return node_manager.global_node_directory[hash]

