#!/usr/bin/env python

# @file runner.py
import argparse
import os
import subprocess
import sys

# import python modules from $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.realpath(
        __file__)), '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(os.path.realpath(
            __file__)), "..")), "tools"))
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")
import traci


class Simulation():
    def __init__(self, options, dt=0.1, run_time=100, no_gui=False):
        self._port = 8873  # the port used for communicating with your sumo instance
        self.options = options
        self.dt = dt
        self.run_time = run_time
        self.sumo_process = None
        self.no_gui = no_gui

    def __enter__(self):
        if (self.no_gui):
            sumoBinary = checkBinary('sumo')
        else:
            sumoBinary = checkBinary('sumo-gui')

        print('Starting simulation')
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        self.sumo_process = subprocess.Popen(
            [sumoBinary, "-c", self.options.c, "--step-length", str(self.dt), "--remote-port",
             str(self._port)], stdout=sys.stdout, stderr=sys.stderr)
        print('Initialized traci on port %d' % self._port)

        traci.init(self._port)
        return self

    def __exit__(self, *args):
        traci.close()
        sys.stdout.flush()
        print('Finished simulation')
        self.sumo_process.wait()


# get_options function for SUMO
def sumo_basic_parser():
    parser = argparse.ArgumentParser()
    # parser.add_argument("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    parser.add_argument("-c", type=str, help='path to config file', required=True)
    return parser
