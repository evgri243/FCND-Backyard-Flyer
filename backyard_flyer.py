import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class Phases(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_phase = Phases.MANUAL
        self.calculate_box()

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if (self.flight_phase == Phases.TAKEOFF) or (self.flight_phase == Phases.WAYPOINT):

            distance = np.linalg.norm(self.local_position - self.target_position)
            if distance < 0.2:
                if len(self.all_waypoints) > 0:
                    self.target_position = self.all_waypoints.pop(0)
                    print(self.target_position)
                    self.waypoint_transition()
                else:
                    self.landing_transition()

    def velocity_callback(self):
        if self.flight_phase == Phases.LANDING:
            if (self.global_position[2] - self.global_home[2] < 0.2):
                if (abs(self.local_velocity[2]) < 0.01):
                    self.disarming_transition()


    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_phase == Phases.MANUAL:
            self.arming_transition()
        elif self.flight_phase == Phases.ARMING:
            self.takeoff_transition()
        elif self.flight_phase == Phases.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        self.all_waypoints = [
            np.array([0.0, 10.0, -3.0]),
            np.array([10.0, 10.0, -3.0]),
            np.array([10.0, 0.0, -3.0]),
            np.array([0.0, 0.0, -3.0])
        ]

    def arming_transition(self):
        print("arming transition")
        self.take_control()
        self.arm()

        current_position = self.global_position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_phase = Phases.ARMING

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = -target_altitude
        self.takeoff(target_altitude)
        self.flight_phase = Phases.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")
        self.cmd_position(self.target_position[0], self.target_position[1], -self.target_position[2], 0)
        self.flight_phase == Phases.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_phase = Phases.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_phase = Phases.DISARMING

    def manual_transition(self):
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_phase = Phases.MANUAL

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
