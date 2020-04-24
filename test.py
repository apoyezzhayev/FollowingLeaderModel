from tqdm import tqdm

from utils.simulation import *  # place it before other utils import
from utils.lane import LanesSimulation

WEGREEN = "rrrrGGggrrrrGGgg"
PROGRAM = [WEGREEN]
NSGREEN = "GGGgrrrrGGGgrrrr"


# Q = veh / s
# rho veh / m
# V = Q / rho
# alpha = -occupancy d_mean_v / d_mean_opccupancy
# alpha = -rho d_v/d_rho


def run(run_time):
    programPointer = 0  # initiates at start # len(PROGRAM) - 1 # initiates at end
    step = 0
    flow_count = 0
    first_car = True
    prev_veh_id = ' '
    leaving_times = []
    car_speeds = []

    with Simulation(options, run_time=run_time, no_gui=True, dt=1) as sim:
        lanes_sim = LanesSimulation(1e-5)
        for step, t in sim:
            # programPointer = (step * int(1 / sim.dt)) % len(PROGRAM)
            # sensor_data = traci.inductionloop.getVehicleData("sensor")
            lanes_sim.step()

            if step % 10 == 0:
                alphas = lanes_sim.alphas
                for l_id, l in lanes_sim.state.items():
                    tqdm.write("%d| Line `%s`: a=%.2f, v=%.2f, r=%.2f" % (step, l_id, alphas[l_id], l.v, l.rho))

        # if len(sensor_data) != 0:
        #     if first_car:  # if its the first car, record the time that it comes in
        #         first_time = sensor_data[0][2]
        #         tqdm.write('First time %.2f' % first_time)
        #         first_car = False
        #
        #     veh_id = sensor_data[0][0]
        #     if veh_id != prev_veh_id:  # if the vehicle coming in has a different id than the previous vehicle, count it towards total flow
        #         flow_count += 1
        #         car_speeds.append(traci.inductionloop.getLastStepMeanSpeed("sensor"))
        #
        #     if sensor_data[0][3] != -1:  # if the vehicle is leaving the sensor, record the time it left
        #         leaving_times.append(sensor_data[0][2])
        #     prev_veh_id = veh_id

        # for l in lanes_sim.lanes:
        lanes_sim.lanes[0].fundamental_diagram()

    print(leaving_times)


if __name__ == "__main__":
    parser = sumo_basic_parser()
    options = parser.parse_args('-c single_lane/hello.sumocfg'.split(' '))
    run(run_time=40)
