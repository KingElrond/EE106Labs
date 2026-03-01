def neighbors(current):
    # define the list of 4 neighbors
    neighbors = [[1,0],[0,1],[0,-1],[-1,0]]
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    #manhattan Distance
    return abs(candidate[0] - goal[0]) + abs(candidate[1] - goal[1])

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2
    path = []
    g0 = 0
    h0 = heuristic_distance(start, goal)
    f0 = g0 + h0
    startNode = (f0, start)
    unexplored = [startNode] #unexplored grids
    explored = [] #explored grids
    parentcost = {}
    parentcost[start] = (None, 0)
    
    #while unexplored is not empty
    while len(unexplored)!=0:
        unexplored.sort() #sorts all grids by lowest f
        
        current_f, current = unexplored.pop(0)
        if current in explored: #check if grid already explored
            continue
        explored.append(current)
        
        if current == goal:
            #set path
            while current != start:
                path.append(current)
                current = parentcost[current][0]
            path.reverse()
            return path
        
        #get g and expand
        current_g = parentcost[current][1]
        for candidate in neighbors(current):
            #filter out invalid candidates
            if candidate in obstacles:
                continue
            if candidate in explored:
                continue
            cand_g = current_g + 1
            cand_f = cand_g + heuristic_distance(candidate, goal)
            
            #add node to list if not in parentcost or found cheaper path
            if candidate not in parentcost or cand_g < parentcost[candidate][1]:
                parentcost[candidate] = (current, cand_g)
                unexplored.append((cand_f, candidate))
                
                
    return path