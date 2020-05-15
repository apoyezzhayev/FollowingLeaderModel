from utils.simulation import *  # place it before other utils import
from utils.lane import LaneSimulation

import traci.constants as tc
import traci

# constants are here: https://github.com/eclipse/sumo/blob/master/tools/traci/constants.py

# Q = veh / s
# rho veh / m
# V = Q / rho
# alpha = -occupancy d_mean_v / d_mean_opccupancy
# alpha = -rho d_v/d_rho
from utils.vehicles import VehiclesSimulation


def run(run_time):
    with Simulation(options, run_time=run_time, no_gui=False, dt=0.1) as sim:
        lanes_sim = LaneSimulation(dv_dr='dvdr.pickle')
        vehicles_sim = VehiclesSimulation(lanes_sim.lanes, model='platoon')
        # Get lanes
        for step, t in sim:
            # print('Step %d' % step)
            lanes_sim.step()
            vehicles_sim.step()
            # Add new vehciles to subscription

            # positions = traci.vehicle.getAllSubscriptionResults()
            # occupancies = traci.lane.getAllSubscriptionResults()
            # print(occupancies)
            # print("step %d" % step)

        #     lanes_sim.step()
        # lanes_sim.save('lanes_sim_data_innopolis.csv')


if __name__ == "__main__":
    parser = sumo_basic_parser()
    # options = parser.parse_args('-c maryland/testmap.sumocfg'.split(' '))
    # options = parser.parse_args('-c ../lab_1/inno.sumocfg'.split(' '))
    options = parser.parse_args('-c single_lane/hello.sumocfg'.split(' '))
    run(run_time=1000)
