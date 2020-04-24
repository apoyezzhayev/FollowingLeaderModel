from tqdm import tqdm
from utils.simulation import *

from traci import lane


WEGREEN = "rrrrGGggrrrrGGgg"
PROGRAM = [WEGREEN]
NSGREEN = "GGGgrrrrGGGgrrrr"


# Q = veh / s
# rho veh / m
# V = Q / rho
# alpha = -occupancy d_mean_v / d_mean_opccupancy
# alpha = -rho d_v/d_rho

class LaneState:
    def __init__(self, eps=1e-5):
        self.state = {}
        self.lane_ids = lane.getIDList()
        for l in self.lane_ids:
            self.state[l] = (0, 0, 0)  # speed, rho, alpha

        self.eps = eps

    def step(self):
        for lane_id in self.lane_ids:
            prev_v, prev_rho, prev_alpha = self.state[lane_id]
            rho = lane.getLastStepOccupancy(lane_id)
            v = lane.getLastStepMeanSpeed(lane_id)
            d_rho = prev_rho - rho
            d_v = prev_v - v
            alpha = -rho * d_v / (d_rho + self.eps)
            self.state[lane_id] = (v, rho, alpha)


def run(run_time=36):
    programPointer = 0  # initiates at start # len(PROGRAM) - 1 # initiates at end
    step = 0
    flow_count = 0
    first_car = True
    prev_veh_id = ' '
    leaving_times = []
    car_speeds = []

    with Simulation(options, run_time=run_time, no_gui=True, dt=0.1) as sim:
        lane_state = LaneState(1e-5)
        for step in tqdm(range(0, int(sim.run_time * (1 / sim.dt)))):
            traci.simulationStep()
            programPointer = (step * int(1 / sim.dt)) % len(PROGRAM)
            sensor_data = traci.inductionloop.getVehicleData("sensor")
            lane_state.step()
            if step % 10 == 0:
                for l_id, (v, rho, alpha) in lane_state.state.items():
                    tqdm.write("%d| Line `%s`: a=%.2f, v=%.2f, r=%.2f" % (step, l_id, alpha, v, rho))

        if len(sensor_data) != 0:
            if first_car:  # if its the first car, record the time that it comes in
                first_time = sensor_data[0][2]
                tqdm.write('First time %.2f' % first_time)
                first_car = False

            veh_id = sensor_data[0][0]
            if veh_id != prev_veh_id:  # if the vehicle coming in has a different id than the previous vehicle, count it towards total flow
                flow_count += 1
                car_speeds.append(traci.inductionloop.getLastStepMeanSpeed("sensor"))

            if sensor_data[0][3] != -1:  # if the vehicle is leaving the sensor, record the time it left
                leaving_times.append(sensor_data[0][2])
            prev_veh_id = veh_id

    print(leaving_times)


if __name__ == "__main__":
    parser = sumo_basic_parser()
    options = parser.parse_args('-c single_lane/hello.sumocfg'.split(' '))
    run()

    #
    # traci.vehicle.add("1", "route1", typeID="HDV", depart="0", departPos="30", departSpeed="20")
    # traci.vehicle.add("2", "route1", typeID="HDV", depart="0", departPos="15", departSpeed="20")
    # traci.vehicle.add("3", "route1", typeID="HDV", depart="0", departPos="0", departSpeed="20")
    # traci.vehicle.setSpeed("1", 20)
