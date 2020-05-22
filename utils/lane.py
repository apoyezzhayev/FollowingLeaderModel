import pickle

from traci import constants as tc
from traci import lane as tl


class Lane:
    def __init__(self, id, dv_dr_func=None):
        self.id = id
        self._buffer_state = None  # v, rho, q
        self._length = tl.getLength(self.id)
        self.dv_dr = dv_dr_func

    def update(self, state):
        mean_speed = state[tc.LAST_STEP_MEAN_SPEED]
        num = state[tc.LAST_STEP_VEHICLE_NUMBER]
        mean_density = num / self._length / 1000.
        mean_flow = mean_density * mean_speed
        self._buffer_state = (mean_speed, mean_density, mean_flow)

    @property
    def alpha(self):
        density = self._buffer_state[1]
        if self.dv_dr is None:
            return 1000
        else:
            return -density * self.dv_dr(density)


class LaneSimulation:
    def __init__(self, dv_dr=None, lane_ids=None):
        if dv_dr is not None:
            with open(dv_dr, 'r') as f:
                dv_dr = pickle.load(f)
        self.dv_dr = dv_dr

        self.lanes = {id: Lane(id, dv_dr.get(id)) for id in (tl.getIDList() if lane_ids is None else lane_ids)}

        for lane_id in self.lanes.keys():
            tl.subscribe(lane_id, [tc.LAST_STEP_MEAN_SPEED, tc.LAST_STEP_VEHICLE_NUMBER])

    def step(self):
        out = tl.getAllSubscriptionResults()
        for lane_id, res in out.items():
            self.lanes[lane_id].update(res)
