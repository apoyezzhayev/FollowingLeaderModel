import numpy as np
import traci
from traci import vehicle

EPS = 1e-6

class VehiclesSimulation:
    def __init__(self, lanes, dt=0.05, model='gm'):
        """
        Updates states of all lanes
        :param lanes: all lanes - dict
        :param dt: simulation interval
        :param model: name of model: `gm` or `platoon`
        """
        self.dt = dt
        self.model = model
        self.lanes = lanes
        self.vehicles = {}

    def step(self):
        '''
        Advance one simulation step.

        vehicles - array of vehicles
        dt - length of simulation step in seconds
        '''
        self.vehicles = {id: Vehicle(id, self.dt) for id in traci.vehicle.getIDList()}

        for veh_id, vehicle in self.vehicles.items():
            leader = self.get_leader(vehicle)
            if self.model == 'gm':
                vehicle.step_gm(leader, self.lanes[vehicle.lane_id].alpha)
            elif self.model == 'platoon':
                vehicle.step_platoon(leader)


    def get_leader(self, vehicle):
        leader = vehicle.leader_id
        if leader is None:
            return leader
        else:
            leader_id, dest = leader
        if leader_id == '': leader_id = None
        leader = self.vehicles.get(leader_id, None)
        return leader


class Vehicle:
    def __init__(self, id, dt=0.05, min_gap=4, tau=2.05):
        '''
        id - vehicle number
        x - position in meters
        l - car length in meters
        a - acceleration in m/s^2
        b - deceleration in m/s^2
        v_max - maximal speed in m/s
        g_min - minimal distance gap in meters
        tau - reaction time in seconds
        model - model type: 'k' = Krauss; 'i' = IDM; 'g' = Gipps; 'h' = Helly
        '''

        self.id = id
        self.l = vehicle.getLength(self.id)
        self.max_a = vehicle.getAccel(self.id)
        self.max_b = vehicle.getDecel(self.id)
        self.min_gap = min_gap
        self.tau = tau

        self.dt = dt

    @property
    def leader_id(self):
        return vehicle.getLeader(self.id, dist=50)

    @property
    def v_max(self):
        return vehicle.getAllowedSpeed(self.id)

    @property
    def lane_id(self):
        return vehicle.getLaneID(self.id)

    @property
    def x(self):
        return vehicle.getLanePosition(self.id)
        # return vehicle.getPosition(self.id)

    # @x.setter
    # def x(self, x):
    #     vehicle.moveTo(self.id, x, self.lane_id)

    @property
    def v(self):
        return traci.vehicle.getSpeed(self.id)

    def set_v(self, v):
        traci.vehicle.setSpeed(self.id, v)

    @property
    def a_actual(self):
        return vehicle.getAcceleration(self.id)

    def step_gm(self, leader=None, alpha=13):
        if leader is None:
            self.set_v(self.v_max)  # max permitted speed with safety rules
            return
        else:
            x_l, v_l = leader.x, leader.v

            gap = x_l - self.x - self.l
            a_estimate = min(alpha * (v_l - self.v) / gap, self.max_a)
            new_v = self.v + a_estimate * self.dt
            new_v = max(0., new_v)
            self.set_v(new_v)

            # We don't need to set it manually, SUMO will do it for us
            # a_actual = float(new_v - self.v) / self.dt  # if one wants to write it
            # self.x = self.x + ((self.v + new_v) / 2) * self.dt

    def step_platoon(self, leader=None):
        '''
        leader - vehicle in front, if such exists.
        dt - size of the simulation step in seconds.
        '''
        if leader is None:
            self.set_v(-1)  # max permitted speed with safety rules
            return
        else:
            x_l, v_l = leader.x, leader.v
            alpha = 2
            beta = 2
            c = 1  # c% ACC, (1-c)% IIDM
            b = self.max_b

            a_bar = np.min([leader.a_actual, self.max_a])
            gap = x_l - self.x - self.l

            a_cah = 0
            if v_l > 0:
                a_cah = (self.v ** 2 * a_bar) / (v_l ** 2 - alpha * gap * a_bar)

                if v_l * (self.v - v_l) > -alpha * gap * a_bar:
                    theta = 0
                    if self.v - v_l >= 0:
                        theta = 1
                    a_cah = a_bar - theta * (self.v - v_l) ** 2 / (beta * gap)

            gap_desired = self.min_gap + np.max(
                [0, self.v * self.tau + self.v * (self.v - v_l) / (2 * np.sqrt(self.max_a * self.max_b))])

            p1, p2 = 4, 8
            a_free = self.max_a * (1 - (self.v / (self.v_max + EPS)) ** p1)

            z = gap_desired / gap
            if z >= 1:
                a_iidm = self.max_a * (1 - z ** p2)
            else:
                a_iidm = a_free * (1 - z ** (p2 * self.max_a / (a_free + EPS)))
            a_cacc = a_iidm
            if gap <= self.min_gap:
                a_cacc = np.min([a_cacc, (v_l - self.v) / self.dt])
            new_v = self.v + a_cacc * self.dt

            # We don't need to set it manually
            # self.gap = gap
            # self.x = self.x + ((self.v + new_v) / 2) * self.dt
            # self.a_actual = float(new_v - self.v) / self.dt
            self.set_v(new_v)


