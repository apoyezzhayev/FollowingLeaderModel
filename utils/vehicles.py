import numpy as np
import traci
from traci import vehicle


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
    def __init__(self, id, dt=0.05, min_gap=4):
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
        self.min_gap = min_gap

        self.dt = dt

    @property
    def leader_id(self):
        return vehicle.getLeader(self.id, dist=300)

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
        return vehicle.getSpeed(self.id)

    @v.setter
    def v(self, v):
        vehicle.setSpeed(self.id, v)

    def step_gm(self, leader=None, alpha=1):
        if leader is None:
            self.v = -1  # max permitted speed with safety rules
            return
        else:
            x_l, v_l = leader.x, leader.v

            gap = x_l - self.x - self.l
            a_estimate = min(alpha * (v_l - self.v) / gap, self.max_a)
            new_v = self.v + a_estimate * self.dt
            new_v = np.max([0, new_v])

            a_actual = float(new_v - self.v) / self.dt  # if one wants to write it
            self.v = new_v
            # We don't need to set it manually, SUMO will do it for us
            # self.x = self.x + ((self.v + new_v) / 2) * self.dt

    def step_platoon(self, leader=None):
        pass
