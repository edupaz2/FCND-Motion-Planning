import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from sampling import Sampler
import numpy.linalg as LA
from sklearn.neighbors import KDTree


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)


    def can_connect(self, n1, n2):
        l = LineString([n1, n2])
        for p in polygons:
            if p.crosses(l) and p.height >= min(n1[2], n2[2]):
                return False
        return True

    def create_graph(self, nodes, k):
        g = nx.Graph()
        tree = KDTree(nodes)
        for n1 in nodes:
            # for each node connect try to connect to k nearest nodes
            idxs = tree.query([n1], k, return_distance=False)[0]
            
            for idx in idxs:
                n2 = nodes[idx]
                if n2 == n1:
                    continue
                    
                if can_connect(n1, n2):
                    g.add_edge(n1, n2, weight=1)
        return g

    def a_star_for_graphs(self, graph, heuristic, start, goal):
        """Modified A* to work with NetworkX graphs."""
        
        path = []
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False
        
        while not queue.empty():
            item = queue.get()
            current_cost = item[0]
            current_node = item[1]

            if current_node == goal:        
                print('Found a path.')
                found = True
                break
            else:
                for next_node in graph[current_node]:
                    cost = graph.edges[current_node, next_node]['weight']
                    new_cost = current_cost + cost + heuristic(next_node, goal)
                    
                    if next_node not in visited:                
                        visited.add(next_node)               
                        queue.put((new_cost, next_node))
                        
                        branch[next_node] = (new_cost, current_node)
                 
        path = []
        path_cost = 0
        if found:
            
            # retrace steps
            path = []
            n = goal
            path_cost = branch[n][0]
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
                
        return path[::-1], path_cost

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        #self.target_position[2] = TARGET_ALTITUDE

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().split(',')
            lat0, lon0 = float(first_line[0].split(' ')[-1]), float(first_line[1].split(' ')[-1])

            # set home position to (lon0, lat0, 0)
            self.set_home_position(lon0, lat0, 0)  # set the current location to be the home position

        # TODO: retrieve current global position
        # global_position = [self.longitude, self.latitude, self.altitude]
        # convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        print('Sampling')
        t0 = time.time()
        sampler = Sampler(data)
        polygons = sampler.polygons
        nodes = sampler.sample(500)
        print('Sampling took {0} for {1}'.format(time.time()-t0, len(nodes)))
        print('****')
        print('Creating graph')
        t0 = time.time()
        g = self.create_graph(nodes, 10)
        print('graph took {0} seconds to build'.format(time.time()-t0))
        print('Number of edges: ', len(g.edges))
        print('****')
        print("A*")
        #start = list(g.nodes)[0]
        start = (self.longitude, self.latitude, TARGET_ALTITUDE)
        k = np.random.randint(len(g.nodes))
        print(k, len(g.nodes))
        goal = list(g.nodes)[k]

        path, cost = self.a_star_for_graphs(g, heuristic, start, goal)
        print('A* took {0}, length: {1} path: {2}'.format(len(path), path))

        # Convert path to waypoints
        waypoints = [[p[0], p[1], p[2], 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

        # ########################3
        # # Define a grid for a particular altitude and safety margin around obstacles
        # grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        # print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # # Define starting point on the grid (this is just grid center)
        # grid_start = (-north_offset, -east_offset)
        # # TODO: convert start position to current position rather than map center
        
        # # Set goal as some arbitrary position on the grid
        # grid_goal = (-north_offset + 10, -east_offset + 10)
        # # TODO: adapt to set goal as latitude / longitude position and convert

        # # Run A* to find a path from start to goal
        # # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # # or move to a different search space such as a graph (not done here)
        # print('Local Start and Goal: ', grid_start, grid_goal)
        # path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # # TODO: prune path to minimize number of waypoints
        # # TODO (if you're feeling ambitious): Try a different approach altogether!

        # # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # # Set self.waypoints
        # self.waypoints = waypoints
        # # TODO: send waypoints to sim (this is just for visualization of waypoints)
        # self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
