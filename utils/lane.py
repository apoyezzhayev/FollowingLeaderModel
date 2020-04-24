from collections import defaultdict
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np
from traci import lane


class Lane:
    _state_cls = namedtuple('LaneState', ['v', 'rho', 'Q'])

    def __init__(self, id, eps=1e-5):
        self.id = id
        self.eps = eps
        self._state = []
        self._prev_vehicles = set()

    def step(self):
        vehicles = set(lane.getLastStepVehicleIDs(self.id))
        new_vehicles = vehicles.difference(self._prev_vehicles)
        self._prev_vehicles = vehicles

        self._state.append(self._state_cls(lane.getLastStepMeanSpeed(self.id),
                                           lane.getLastStepOccupancy(self.id),
                                           len(new_vehicles)))

    @property
    def alpha(self):
        if len(self._state) == 1:
            return 0
        else:
            v, rho = self._state[-1].v, self._state[-1].rho
            d_rho = self._state[-2].rho - rho
            d_v = self._state[-2].v - v
            alpha = -rho * d_v / (d_rho + self.eps)
            return alpha

    @property
    def cur_state(self):
        return self._state[-1]

    @property
    def state_history(self):
        return self._state

    def fundamental_diagram(self):
        states = np.array(self._state)
        v, rho, Q = tuple([states[:, i] for i in range(states.shape[-1])])
        plt.scatter(rho, v)
        plt.xlabel('rho, %occupied ~(veh/m)')
        plt.ylabel('v, m/s')
        plt.show()
        plt.scatter(rho, Q)
        plt.xlabel('rho, %occupied ~(veh/m)')
        plt.ylabel('Q, veh/s')
        plt.show()


class LanesSimulation:
    def __init__(self, eps=1e-5, dt=10):
        self.dt = dt
        self._state = defaultdict(list)
        self.lanes = [Lane(id, eps) for id in lane.getIDList()]

    def step(self):
        for lane in self.lanes: lane.step()

    @property
    def alphas(self):
        return {lane.id: lane.alpha for lane in self.lanes}

    @property
    def state(self):
        return {lane.id: lane.cur_state for lane in self.lanes}
