# code writing by Junle Yuan, Yanwen Xu, Adam Carter
import math
from math import inf, sqrt
from heapq import heappop, heappush

"""
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
"""
def find_path (source_point, destination_point, mesh):

    print("start:", source_point)
    print("end:", destination_point)

    firstbox = None
    lastbox = None

    if source_point == destination_point:
        print("start and end are the same")
        return [],[]

    for i in mesh['boxes']:
        if source_point[0] >= i[0] and source_point[0] <= i[1] and source_point[1] >= i[2] and source_point[1] <= i[3]:

            firstbox = i
        
        if destination_point[0] >= i[0] and destination_point[0] <= i[1] and destination_point[1] >= i[2] and destination_point[1] <= i[3]:
    
            lastbox = i

    if firstbox is None:
        print("source_point is not in correct area")
        return [], []

    if lastbox is None:
        print("destination_point is not in correct area")
        return [], []
    
    print(firstbox)
    print(lastbox)
    
    frontier = []
    #we are pushing start and end to keep track of which end we are coming from
    heappush(frontier, ( 0,firstbox, 'start'))
    heappush(frontier, ( 0,lastbox, 'end'))
                                           
    came_from = dict()
    cost_so_far = dict()
    came_from[firstbox] = None
    cost_so_far[firstbox] = 0

    back_came_from = dict()
    back_cost_so_far = dict()
    back_came_from[lastbox] = None
    back_cost_so_far[lastbox] = 0 

    path = []
    boxes = []
    
    while frontier:
        priority, cell, direction = heappop(frontier)
        
        if cell in came_from and cell in back_came_from :
            forward_boxes = path_to_cell(cell,came_from)

            backward_boxes = path_to_cell(cell,back_came_from)

            backward_boxes.pop()
            backward_boxes = backward_boxes[::-1]
            
            boxes = forward_boxes + backward_boxes
            
            "stright line alg: "
            cur_point = source_point
            path.append(source_point)
            slope = (source_point[1]-destination_point[1])/(source_point[0]-destination_point[0])
            b = source_point[1]-source_point[0]*slope
            check = False
            for n in range(source_point[0],destination_point[0]):
                m = (n*slope)+b
                
                check = False
                for box in boxes:
                    if (n >= box[0]) and (n <= box[1]) and (m >= box[2]) and (m <= box[3]):
                        
                        check = True
                        break
                if check is False:
                    break
            if check is True:
                path.append(destination_point)
                return path, boxes
            "end of stright line alg: "

            for n in range(1,len(boxes),1):
                cur_box = boxes[n]
                cur_point = get_best_point(cur_point,cur_box)
                path.append(cur_point)
            
            path.append(destination_point)
            
            return path, boxes
        
        if direction == 'start':
            tempbox = mesh['adj'].get(cell)
            for next in tempbox:
                new_cost = priority + getdist((cell[0]+cell[1])/2,(cell[2]+cell[3])/2,(next[0]+next[1])/2,(next[2]+next[3])/2)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    new_cost += getdist((next[0]+next[1])/2,(next[2]+next[3])/2,(lastbox[0]+lastbox[1])/2,(lastbox[2]+lastbox[3])/2)
                    came_from[next] = cell
                    heappush(frontier,(new_cost,next,'start'))
        else:
            tempbox = mesh['adj'].get(cell)
            for next in tempbox:
                new_cost = priority + getdist((cell[0]+cell[1])/2,(cell[2]+cell[3])/2,(next[0]+next[1])/2,(next[2]+next[3])/2)
                if next not in back_cost_so_far or new_cost < back_cost_so_far[next]:
                    back_cost_so_far[next] = new_cost
                    new_cost += getdist((next[0]+next[1])/2,(next[2]+next[3])/2,(firstbox[0]+firstbox[1])/2,(firstbox[2]+firstbox[3])/2)
                    back_came_from[next] = cell
                    heappush(frontier,(new_cost,next,'end'))
             
    print("No Path")
    return [], []


"""
    returns the best point to try going to next

    Args:
        current: the current point that we are trying to check
        next_box: the box that is adjacent to the current box

    Returns:
        The closest point to where we currently were
"""
def get_best_point(current,next_box):
    if current[0] >= next_box[0] and current[0] <= next_box[1]:

        if  current[1] < next_box[2]:

            best_point = (current[0],next_box[2])

        else:

            best_point = (current[0],next_box[3])

    elif current[1] >= next_box[2] and current[1] <= next_box[3]:

        if  current[0] < next_box[0]:

            best_point = (next_box[0],current[1])

        else:

            best_point = (next_box[1],current[1])

    elif current[0] < next_box[0]:

        if current[1] < next_box[2]:
            best_point = (next_box[0],next_box[2])
        else:
            best_point = (next_box[0],next_box[3])
    else:

        if current[1] < next_box[2]:
            best_point = (next_box[1],next_box[2])
        else:
            best_point = (next_box[1],next_box[3])

    return best_point

"""
    Returns the path to the passed cell

    Args:
        cell: the cell that we are trying to go to
        paths: the paths that can be taken to the cell

    Returns:
        a path to a target cell
"""
def path_to_cell(cell, paths):
    if cell == None:
        return []
    if cell == []:
        return []
    return path_to_cell(paths[cell], paths) + [cell]

"""
    gets the distance between given points

    Args:
        x1/y1 is the coordinates of the first point
        x2/y2 is the coordinates of the second point

    Returns:
        distance between given points
"""  
def getdist (x1, y1, x2, y2):
    return math.sqrt(abs(x1-x2)**2 + abs(y1-y2)**2)

"""
    returns the box from the given arguments

    Args:
        item is a single square on the mesh
        mesh is the nav mesh

    Returns:
        either the box that matches from the item and mesh or none if it doesn't exist
"""
def getbox(item,mesh):
    for cur_box in mesh['boxes']:
        if item[0] >= cur_box[0] and item[0] <= cur_box[1] and item[1] >= cur_box[2] and item[1] <= cur_box[3]:
            x = cur_box
            return x

    return None