import argparse
import time
import sys
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star_graph, heuristic
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

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
    pkl_filename = 'graph.random_sampling.p'
    with open(pkl_filename, "rb") as pfile:
        dist_pickle = pickle.load(pfile)
        g = dist_pickle['graph']
        print('Graph with {0} nodes and {1} edges took {2} secs'.format(len(g.nodes), len(g.edges), time.time()-t0))
        return g

class MotionPlanning(Drone):

    def __init__(self, connection, graph):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.graph = graph

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                # send waypoints to sim (this is just for visualization of waypoints)
                #self.send_waypoints()
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
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
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
        SAFETY_DISTANCE = 5

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
        drone_location = (self.local_position[0], self.local_position[1], TARGET_ALTITUDE)
        print('Finding the nearest start node')

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

        self.target_position[2] = TARGET_ALTITUDE

        # Select a random goal node
        rnd_goal = np.random.randint(len(self.graph.nodes))
        goal = list(self.graph.nodes)[rnd_goal]
        print('Selecting random goal: ', rnd_goal)

        print('****')
        print("A*")
        path, cost = a_star_graph(self.graph, heuristic, nearest_start, goal)
        print('A* from {0} to {1} with a length of: {2}'.format(nearest_start, goal, len(path)))

        # Remember to set the Altitude, waypoints are higher
        # Cull path: eliminate unnecessary points
        #waypoints = [path[0]]
        #idx = 0
        #while idx < len(path):
            


        # Convert path to waypoints
        waypoints = [[p[0], p[1], p[2], 0] for p in path]

        # Define two waypoints with heading = 0 for both
        #wp1 = [n1, e1, a1, 0]
        #wp2 = [n2, e2, a2, 0]
        # Set heading of wp2 based on relative position to wp1
        #wp2[3] = np.arctan2((wp2[1]-wp1[1]), (wp2[0]-wp1[0]))


        # Set self.waypoints
        self.waypoints = waypoints

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

    graph = precompute_graph()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=600)
    drone = MotionPlanning(conn, graph)
    time.sleep(1)

    drone.start()
