from math import pow, sqrt
from collections import namedtuple
import heapq

class Map:

    def __init__(self):
        self.intersections = None
        self.roads = None

def calculate_distance(origin, destination):
    return sqrt(pow((origin[0] - destination[0]),2) + pow((origin[1] - destination[1]),2))


Cost = namedtuple("Cost",['total', 'journey', 'to_goal'])
Path = namedtuple("Path", ['cost', 'intersections', 'previous','frontier'])

def update_path(map: Map, path: Path, new_frontier: int, goal: int):
    traversed_distance = calculate_distance(map.intersections[path.frontier], map.intersections[new_frontier])
    new_path_cost_journey = path.cost.journey + traversed_distance
    new_path_cost_to_goal = calculate_distance(map.intersections[new_frontier], map.intersections[goal])

    new_path_cost_total = new_path_cost_journey + new_path_cost_to_goal

    new_path_intersections = path.intersections + [new_frontier]

    new_path = Path(Cost(new_path_cost_total, new_path_cost_journey, new_path_cost_to_goal), new_path_intersections, path.frontier, new_frontier)
    return new_path

def shortest_path(map, start, goal):
    paths = list()
    path_goal_min_val = float("inf")
    path_goal_min = None

    if start == goal:
        return [start]

    goal_initial_distance = calculate_distance(map.intersections[start], map.intersections[goal])
    path = Path(Cost(goal_initial_distance, 0, goal_initial_distance), [start], start, start)

    heapq.heappush(paths, path)

    while len(paths) >= 1:
        nearest_frontier_path = heapq.heappop(paths)
        for neighbor_road in map.roads[nearest_frontier_path.frontier]:

            if neighbor_road == nearest_frontier_path.previous:
                continue
            else:

                new_path = update_path(map, nearest_frontier_path, neighbor_road, goal)

                if neighbor_road == goal:
                    if new_path.cost.total < path_goal_min_val:
                        path_goal_min_val = new_path.cost.total
                        path_goal_min = new_path.intersections
                else:
                    if path_goal_min is not None: # path_goal_min是已经到达终点了，这句说明还没到终点距离已经比path_goal_min大了，所以不用推进heap里 ( Already found the goal with a path)
                        if new_path.cost.total >= path_goal_min_val:
                            pass    # Path not reached goal and already costly
                        else:  # Cheaper path, keep exploring
                            heapq.heappush(paths, new_path)
                    else:   # Not yet found the goal, keep exploring
                        heapq.heappush(paths, new_path)

    return path_goal_min


#%% Test - Dev- Data Preparations
map_10 = Map()
map_10.intersections = {
    0: [0.7798606835438107, 0.6922727646627362],
    1: [0.7647837074641568, 0.3252670836724646],
    2: [0.7155217893995438, 0.20026498027300055],
    3: [0.7076566826610747, 0.3278339270610988],
    4: [0.8325506249953353, 0.02310946309985762],
    5: [0.49016747075266875, 0.5464878695400415],
    6: [0.8820353070895344, 0.6791919587749445],
    7: [0.46247219371675075, 0.6258061621642713],
    8: [0.11622158839385677, 0.11236327488812581],
    9: [0.1285377678230034, 0.3285840695698353]}

map_10.roads = [
    [7, 6, 5],
    [4, 3, 2],
    [4, 3, 1],
    [5, 4, 1, 2],
    [1, 2, 3],
    [7, 0, 3],
    [0],
    [0, 5],
    [9],
    [8]
]

#%%
result = shortest_path(map=map_10, start=6, goal=5)
print(result)

# Reference
# https://github.com/Axel-Bravo/19_udacity_dsa/blob/master/4_004_Project_Implement_Route_Planner/student_code.py