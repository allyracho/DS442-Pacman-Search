# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
import math

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:


    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    initial = problem.getStartState()  # get initial node
    fringe = util.Stack()  # initialize stack
    visited = []  # initialize list of nodes visited
    solution = []  # initialize solution path
    fringe.push((initial, solution))  # push initial node to frontier

    while not fringe.isEmpty():  # while stack is not empty
        node, path = fringe.pop()  # pop first node
        if problem.isGoalState(node):  # if this node is goal, return solution path
            return path
        if node not in visited:  # check is node has been visited
            visited.append(node)  # if not, add to visited
            adj_node = list(problem.getSuccessors(node))  # get neighbors of node just visited
            for i in adj_node:  # loop over all neighbors
                if i[0] not in visited:  # if node not in visited
                    fringe.push((i[0], path + [i[1]]))  # add to fringe with new solution path


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    initial = problem.getStartState()  # get initial node
    fringe = util.Queue()  # initialize queue
    visited = []  # initialize list of nodes visited
    solution = []  # initialize solution path
    fringe.push((initial, solution))  # push initial node to frontier

    while not fringe.isEmpty():  # while stack is not empty
        node, path = fringe.pop()  # pop first node
        if problem.isGoalState(node):  # if this node is goal, return solution path
            return path
        if node not in visited:  # check is node has been visited
            visited.append(node)  # if not, add to visited
            adj_node = list(problem.getSuccessors(node))  # get neighbors of node just visited
            for i in adj_node:  # loop over all neighbors
                if i[0] not in visited:  # if neighbor node not in visited
                    fringe.push((i[0], path + [i[1]]))  # add to fringe with new solution path


def uniformCostSearch(problem):
    """Search the node of least total cost first."""

    initial = problem.getStartState()  # get initial node
    fringe = util.PriorityQueue()  # initialize queue
    visited = []  # initialize list of nodes visited
    solution = [] # initialize solution path
    priority = 0 # initialize priority
    fringe.push((initial, solution), priority)  # push initial node to frontier

    while not fringe.isEmpty():  # while stack is not empty
        node, path = fringe.pop()  # pop first node
        if problem.isGoalState(node):  # if this node is goal, return solution path
            return path
        if node not in visited:  # check is node has been visited
            visited.append(node)  # if not, add to visited
            adj_node = list(problem.getSuccessors(node))  # get neighbors of node just visited
            for i in adj_node:  # loop over all neighbors
                if i[0] not in visited:  # if node not in visited
                    cost = problem.getCostOfActions(path + [i[1]])
                    fringe.push((i[0], (path + [i[1]])), cost)
                elif i[0] in fringe.heap:  # else if, in frontier with higher cost
                    cost = problem.getCostOfActions(path + [i[1]])
                    if fringe.count > cost:  # if the fringe priority is higher
                        fringe.update((i[0], (path + [i[1]])), cost)  # update fringe node with neighbor node


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0



def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    initial = problem.getStartState()  # get initial node
    fringe = util.PriorityQueue()  # initialize queue
    visited = []  # initialize list of nodes visited
    solution = [] # initialize solution path
    priority = 0 # initialize priority
    fringe.push((initial, solution), priority)  # push initial node to frontier

    while not fringe.isEmpty():  # while stack is not empty
        node, path = fringe.pop()  # pop first node
        if problem.isGoalState(node):  # if this node is goal, return solution path
            return path
        if node not in visited:  # check is node has been visited
            visited.append(node)  # if not, add to visited
            adj_node = list(problem.getSuccessors(node))  # get neighbors of node just visited
            for i in adj_node:  # loop over all neighbors
                if i[0] not in visited:  # if node not in visited
                    cost = problem.getCostOfActions(path + [i[1]])
                    fringe.push((i[0], (path + [i[1]])), cost+heuristic(i[0], problem)) # push to fringe
                elif i[0] in fringe.heap:  # else if, in frontier with higher cost
                    cost = problem.getCostOfActions(path + [i[1]])
                    fringe.update((i[0], (path + [i[1]])), cost+heuristic(i[0], problem))  # update fringe node with neighbor node




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
