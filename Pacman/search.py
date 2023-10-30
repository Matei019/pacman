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

    def expand(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (child,
        action, stepCost), where 'child' is a child to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that child.
        """
        util.raiseNotDefined()

    def getActions(self, state):
        """
          state: Search state

        For a given state, this should return a list of possible actions.
        """
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        """
          state: Search state
          action: action taken at state.
          next_state: next Search state after taking action.

        For a given state, this should return the cost of the (s, a, s') transition.
        """
        util.raiseNotDefined()

    def getNextState(self, state, action):
        """
          state: Search state
          action: action taken at state

        For a given state, this should return the next state after taking action from state.
        """
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    search_nodes = util.Stack()  ## folosim o stiva pentru a stoca nodurile de cautat
    visited_nodes = set()   ## folosim un set pentru a stoca nodurile vizitate

    # Push the start state along with an empty list of actions
    search_nodes.push((problem.getStartState(), []))   ### punem pe stiva punctul de start si o lista care sa tina istoricul

    while not search_nodes.isEmpty():    ### cat timp stiva nu este goala
        state, actions = search_nodes.pop()   ### punem in state starea curenta si in actions lista istoricului actiunilor

        if problem.isGoalState(state):  ### daca am ajuns la tinta noastra, returnam lista de actiuni
            return actions

        if state not in visited_nodes:
            visited_nodes.add(state)   ### daca starea curenta nu a mai fost vizitata, o adaugam in setul cu vizitate

            # Use problem.expand to get successors
            successors = problem.expand(state) ### bagam in successors setul child, action, stepCost
            for successor, action, stepCost in successors:
                new_actions = actions + [action]   ### se adauga actiunile viitoare
                search_nodes.push((successor, new_actions))

    return []

    util.raiseNotDefined()

def breadthFirstSearch(problem): 
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    search_nodes = util.Queue()  ## folosim o coada pentru a stoca nodurile de cautat
    visited_nodes = set()   ## folosim un set pentru a stoca nodurile vizitate

    # Push the start state along with an empty list of actions
    search_nodes.push((problem.getStartState(), []))   ### punem in coada punctul de start si o lista care sa tina istoricul

    while not search_nodes.isEmpty():    ### cat timp coada nu este goala
        state, actions = search_nodes.pop()   ### punem in state starea curenta si in actions lista istoricului actiunilor
        visited_nodes.add(state)   ### adaugam starea in setul cu vizitate
        if problem.isGoalState(state):  ### daca am ajuns la tinta noastra, returnam lista de actiuni
            return actions

            # Use problem.expand to get successors
        successors = problem.expand(state) ### bagam in successors setul child, action, stepCost
        for successor, action, _ in successors:#nu ne intereseaza stepCost deci putem folosi _ , pt a arata ca nu vom utiliza respectivul atribut
                if successor not in visited_nodes and all(successor != node[0] for node in search_nodes.list):#verificam daca succesorul gasit nu se afla nici in visited_nodes si nici in search_nodes
                    search_nodes.push((successor, actions + [action]))  # Add the successor to search_nodes with its path , asta decat daca conditia de sus e respectata
                

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    searchNodes = util.PriorityQueue() #stocam nodurile intr-o coada de prioritati
    visitedNodes = set() #stocam nodurile vizitate intr-un set

    startNode = problem.getStartState() #extragem starea de inceput din "problem" (coordonatele x si y ale pozitiei de inceput din graf)
    startHeuristic = heuristic(startNode, problem) #calculam euristica primului nod fata de finish(nodul la care trebuie sa ajungem)

    searchNodes.push((startNode, [], 0), startHeuristic) #punem in coada de prioritati nodul de inceput, impreuna cu o lista goala si costul pana la el,
                                                         #iar ca al doilea parametru punem euristica aleasa
    solution = [] # creeam o lista goala in care vom stoca directiile ce vor rezulta la solutia finala(directii = N, S, E, V; in care va trebui sa mearga pacman)

    while not searchNodes.isEmpty(): #cat timp coada de noduri nu este goala vom extrage cate un element pe rand
        currentNode, solution, cost = searchNodes.pop() #salvam in "currentNode" coordonatele nodului nou extras din "searchNodes", in "solution" lista curenta de noduri care duc
                                                        #la solutie, iar in "cost" costul pana la acel nod (dar nu il vom folosim)
        if problem.isGoalState(currentNode): #verificam daca "currentNode" este nodul la care dorim sa ajungem(in cazul nostru fructul la care trebuie sa mearga pacman)
            return solution #daca am ajuns la fruct(nodul la care dorim sa ajungem), atunci returnam "solution"(lista ce contine nodurile prin care trebuie sa trecem)
        
        #daca "currentNode" nu este nodul la care dorim sa ajungem, atunci va continua executia programului aici
        if currentNode not in visitedNodes: #verificam daca nu am mai trecut prin "currentNode" la un moment anterior(daca nu se afla in "visitedNode")
            visitedNodes.add(currentNode) #daca nu am trecut inca prin el, il marcam ca si vizitat si continuam operatiile necesare asupra lui

            for coordinates, direction, nextCost in problem.expand(currentNode): #parcurgem toti vecinii nodului curent, extragand coordonatele("coordinates"), directia lui
                                                                                 #("direction") si costul pana la el("nextCost", pe care nu il vom folosi)
                if coordinates not in visitedNodes: #daca nu am mai trecut prin acest vecin inainte, atunci putem lucra cu el 
                    actionsList = list(solution) #"actionList" salveaza lista curenta "solution" ce contine directiile curente ce formeaza solutia finala
                    actionsList += [direction] #iar aici adaugam la lista noua directie descoperita

                    actionsCost = problem.getCostOfActionSequence(actionsList) #salvam in "actionsCost" costul total obtinut pana acum de "actionList"(costul curent de la nodul
                                                                               #de start pana la nodul "coordinates")
                    heuristicProblem = heuristic(coordinates, problem) #calculam euristica nodului "coordinates" fata de finish(nodul la care trebuie sa ajungem)

                    searchNodes.push((coordinates, actionsList, 1), (actionsCost + heuristicProblem)) #adaugam la coada de prioritati nodul "coordinates", "actionList"
                                                                                                      #(lista ce contine solutia curenta), si costul 1(euristica, costul este 1),
                                                                                                      #iar ca al doilea parametru suma dintre costul curent si euristica nou calculata

    return [] #in cazul in care nu avem noduri in coada de prioritati sau nu se poate gasi o solutie la problema, atunci vom returna o lista goala


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
