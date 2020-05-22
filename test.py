from utils.simulation import *  # place it before other utils import
from utils.lane import LaneSimulation

import traci.constants as tc
import traci
from tqdm import tqdm

# constants are here: https://github.com/eclipse/sumo/blob/master/tools/traci/constants.py

# Q = veh / s
# rho veh / m
# V = Q / rho
# alpha = -occupancy d_mean_v / d_mean_opccupancy
# alpha = -rho d_v/d_rho
from utils.vehicles import VehiclesSimulation


def run(run_time, opts):
    with Simulation(opts, run_time=run_time, no_gui=opts.nogui, dt=0.1) as sim:
        lanes_sim = LaneSimulation(dv_dr='dvdr_gm.pickle')
        vehicles_sim = VehiclesSimulation(lanes_sim.lanes, model=opts.m)
        # Get lanes
        for step, t in sim:
            if (step % 100) == 0:
                tqdm.write('Step %d: number of vehicles = %d' % (step, traci.vehicle.getIDCount()))
            lanes_sim.step()
            vehicles_sim.step()


if __name__ == "__main__":
    parser = sumo_basic_parser()
    options = parser.parse_args()
    run(run_time=options.s, opts=options)
