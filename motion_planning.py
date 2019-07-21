import argparse
import time
import sys
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star_graph, heuristic, FLYING_ALTITUDE, SAFETY_DISTANCE
from utils import load_graph_from_pickle, print_info

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from bresenham import bresenham

import pickle

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

def precompute_graph():
    print('Precompute Graph')
    t0 = time.time()
    # Load precomputed Voronoi Graph
    #pkl_filename = 'graph.voronoi.p'
    pkl_filename = 'graph.voronoi.p'
    Gr, Cg, north_offset, east_offset = load_graph_from_pickle(pkl_filename)
    print_info(Gr, Cg, north_offset, east_offset)
    return Gr, Cg, north_offset, east_offset

class MotionPlanning(Drone):

    def __init__(self, connection, graph, grid, north_offset, east_offset):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.graph = graph
        self.collision_grid = grid
        self.north_offset = north_offset
        self.east_offset = east_offset

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
            if np.linalg.norm(self.target_position[0:2] - (self.local_position[0:2])) < 2.5:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]-self.target_position[2]) < 0.02:
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
        print("arming transition")
        self.take_control()
        self.arm()
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition to ", self.target_position[2])
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        wp_map_coordinates = self.waypoints.pop(0)
        self.target_position = [wp_map_coordinates[0]+self.north_offset, wp_map_coordinates[1]+self.east_offset, wp_map_coordinates[2], wp_map_coordinates[3]]
        print('Target position in map: {0}, in local: {1}'.format(wp_map_coordinates, self.target_position))
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5.0

        # read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().split(',')
            lat0, lon0 = float(first_line[0].split(' ')[-1]), float(first_line[1].split(' ')[-1])

            # set home position to (lon0, lat0, 0)
            self.set_home_position(lon0, lat0, 0)  # set the current location to be the home position

        # Convert drone global position to local position (relative to global home) using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # Load precomputed graph
        print('Loading graph {0}'.format(len(self.graph.nodes)))


        # Set grid start position at our current drone global position
        drone_location = (self.local_position[0]-self.north_offset, self.local_position[1]-self.east_offset, self.local_position[2])
        print('Drone location local: {0}, Drone location map: {1}'.format(self.local_position, drone_location))

        # Select the nearest node closest to the drone location
        nearest_start = None
        closest_distance = sys.float_info.max
        for n in self.graph.nodes:
            # heuristic is the Euclidean distance:
            distance = heuristic(drone_location, n)
            if distance < closest_distance:
                closest_distance = distance
                nearest_start = n

        if nearest_start == None:
            print('Error while getting closest starting node')
            return
        print('Found starting node = {0}'.format(nearest_start))

        # Select a random goal node
        rnd_goal = np.random.randint(len(self.graph.nodes))
        goal = list(self.graph.nodes)[rnd_goal]
        print('Selecting random goal[{0}]: {1}'.format(rnd_goal, goal))

        print('****')
        path, cost = a_star_graph(self.graph, heuristic, nearest_start, goal)
        print('A* from {0} to {1} with a length of: {2}'.format(nearest_start, goal, len(path)))
        print('A* path: ', path)

        # First waypoint of path
        curr = path[0]

        # Save height for Takeoff
        self.target_position[2] = curr[2]

        # Add first waypoint
        waypoints = [[curr[0], curr[1], curr[2], 0]]

        idx = 1
        while idx < len(path):
            prev_wp = waypoints[len(waypoints)-1]
            # idx indicate a good waypoint candidate to be inserted
            # path[idx] is supposed to be a good candidate already for the A* construction
            # Check following waypoints
            while idx+1 < len(path):
                candidate_wp = path[idx+1]
                hit = False
                cells = list(bresenham(int(prev_wp[0]), int(prev_wp[1]), int(candidate_wp[0]), int(candidate_wp[1])))
                for c in cells:
                    # Check if we're in collision
                    if self.collision_grid[c[0], c[1]] >= FLYING_ALTITUDE + SAFETY_DISTANCE:
                        hit = True
                        break
                # If the path does not hit on obstacle, add it to the list
                if not hit:
                    # It's a good candidate. Update idx
                    idx = idx+1
                else:
                    break

            # Add the candidate using idx
            good_candidate = path[idx]
            curr_wp = [good_candidate[0], good_candidate[1], good_candidate[2], 0]

            # Set heading of curr_wp based on relative position to prev_wp
            heading = np.arctan2((curr_wp[1]-prev_wp[1]), (curr_wp[0]-prev_wp[0]))
            curr_wp[3] = heading

            # Append it to waypoints
            waypoints.append(curr_wp)
            idx = idx+1

        # Set self.waypoints
        self.waypoints = waypoints
        #self.send_waypoints()

        self.flight_state = States.PLANNING

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

    Gr, Cg, noff, eoff = precompute_graph()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600)
    drone = MotionPlanning(conn, Gr, Cg, noff, eoff)
    time.sleep(1)

    drone.start()
