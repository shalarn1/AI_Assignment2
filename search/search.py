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

import util
import node
import sys
import searchAgents

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    [((4,5), "South", 1), (3,5)]

    "*** YOUR CODE HERE ***"
        """
    path_stack = util.Stack()
    path_stack.push((problem.getStartState(), None, [], 0))
    visited = []
    while True:
        if path_stack.isEmpty(): return sys.exit('Failure: no solution')
        curr_state, curr_parent, curr_dir, curr_cost = path_stack.pop()
        visited.append(curr_state)

        for coord, direction, cost in problem.getSuccessors(curr_state):
            if coord not in visited and coord not in path_stack:
                if problem.isGoalState(coord):
                    return curr_dir + [direction]
                path_stack.push((coord, curr_state, curr_dir + [direction], curr_cost + cost))
    return []
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    """
    path_stack = util.Queue()
    path_stack.push((problem.getStartState(), None, [], 0))
    visited = []
    while True:
        if path_stack.isEmpty(): return sys.exit('Failure: no solution')
        curr_state, curr_parent, curr_dir, curr_cost = path_stack.pop()
        visited.append(curr_state)

        for coord, direction, cost in problem.getSuccessors(curr_state):
            if coord not in visited and coord not in path_stack:
                if problem.isGoalState(coord):
                    return curr_dir + [direction]
                path_stack.push((coord, curr_state, curr_dir + [direction], curr_cost + cost))
    return [] """
    util.raiseNotDefined()

def iterativeDeepeningSearch(problem):
    limit = 0
    
    while True:
        result = depthLimitedSearch(problem,limit)
        if result != "end": return result
        limit +=1

    return []
    util.raiseNotDefined()

def depthLimitedSearch(problem,limit=1000):
  
    path_queue = util.Stack()
    path_queue.push((problem.getStartState(), None, [], 0))
    visited = []
    end = False
    
    while True:
        if path_queue.isEmpty(): return "end"          
        curr_state, curr_parent, curr_dir, curr_cost = path_queue.pop()
        visited.append(curr_state)
        if curr_cost == limit: end = True
        else:
            for state, action, cost in problem.getSuccessors(curr_state):
                #ns = node.Node(state, n, action, n.pathcost+cost)         
                if state not in visited and state not in path_queue: 
                    if problem.isGoalState(state): 
                        return curr_dir + [action]
                    path_queue.push((state, curr_state, curr_dir + [action], curr_cost + cost))


"""
    path_stack = util.Stack()
    depth = 0
    while True:
        result = dls(problem, problem.getStartState(), path_stack, 0, depth)
        if result != []:
            return result
        else:
            path_stack = util.Stack()
            path_stack.push((problem.getStartState(), [], []))
            depth += 1

def dls(problem, startState, stack, depth, max_depth):
    while depth <= max_depth:
        stack.push((problem.getStartState(), [], []))
        while not stack.isEmpty():
            curr_state, curr_dir, nodes_visited = stack.pop()

            for coord, direction, cost in problem.getSuccessors(curr_state):
                if not coord in nodes_visited:
                    if problem.isGoalState(coord):
                        return curr_dir + [direction]
                    depth += 1
                    stack.push((coord, curr_dir + [direction], nodes_visited + [curr_state] ))
    return [] """

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
