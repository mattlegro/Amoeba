import math
import numpy as np
from util.orientation import Orientation, relative_location
from util.sequence import Sequence, ControlStep
from util.vec import Vec3
from rlbot.agents.base_agent import SimpleControllerState

class NestedArrayStructure:

    def __init__(self,num_time_steps,num_guess_paths):

        temp_list = [0] * num_guess_paths
        base_array = np.array([0,np.asarray(temp_list).reshape(num_guess_paths,1)],dtype = object)

        for x in range(num_time_steps-1):

            temp_array = base_array
            for y in range(num_guess_paths-1):
                base_array = np.vstack((base_array,temp_array))
            base_array = np.array([0,base_array],dtype = object)


def find_DSM_path(target: Vec3, pos_steps, velocity_steps, L_steps, controls_tracker, num_guess_paths, num_time_steps, time_step):

    turn_step = 2 / (num_guess_paths - 1)
    turn_inputs = [ -1 + turn_step * x for x in range(num_guess_paths)]
    print(f"turn inputs = {turn_inputs}")
    delta_t = .1
    print(f"time step = {time_step}")

    if time_step <= num_time_steps:

        test_velos = [0] * num_guess_paths
        test_positions = test_velos[:]
        test_lagrangians = test_velos[:]
        test_controls = test_velos[:]

        for t in range(len(turn_inputs)):

            # find travel arc at current speed over delta t, update car velocity and location

            # first find distance traveled and store new location for step

            turn_radius = turn_inputs[t] * find_turn_radius(velocity_steps[time_step - 1].length())
            print(f"turn radius = {turn_radius}")
            if turn_radius == 0:
                theta = 0
                d = velocity_steps[time_step - 1].length() * delta_t
            else:
                theta = velocity_steps[time_step - 1].length() * delta_t / abs(turn_radius)
                d = 2 * abs(turn_radius) * math.sin( theta / 2 )
            print(f"step displacement = {d}")
            print(f"theta = {theta}")
            phi = .5 * (math.pi - theta)
            y_disp = d * math.sin(phi)
            if turn_radius < 0:
                x_disp = d * math.cos(phi)
            else:
                x_disp = -1 * d * math.cos(phi)

            # x and y displacement above relative to direction of travel, need relative to field xyz
            # let angle psi be between car velocity and true x_hat
            # and eta the angle between true x_hat and directly to the cars left, car_vx_hat
            # determine quadrant to get appropriate sign for eta

            psi = velocity_steps[time_step-1].ang_to(Vec3(1,0,0))

            if velocity_steps[time_step-1].y >= 0:
                eta = math.pi / 2 - psi
            else:
                eta = 3 * math.pi / 2 - psi

            print(f"eta = {eta}")
            disp_vec = Vec3(x_disp,y_disp,0)
            print(f"disp vec = {disp_vec}")
            field_disp = disp_vec.rotate_xy(eta)
            print(f"field disp = {field_disp}")
            test_positions[t] = pos_steps[time_step - 1] + field_disp

            # find and store velocity vector for step. first, update x,y components due to acceleration.
            # then, rotate velocity vector to correspond with new travel direction based on above turn.

            throttle_accel = find_throttle_accel(velocity_steps[time_step-1].length())
            accel_vec = Vec3( throttle_accel * math.cos(psi), throttle_accel * math.sin(psi), 0)
            new_velo = velocity_steps[time_step - 1] + accel_vec * delta_t

            # if new_velo is greater than achievable max throttle speed, fix

            if new_velo.length() > 1410:
                new_velo.rescale(1410)
            
            # rotate to correct orientation

            if turn_radius < 0:
                theta = theta * -1
            test_velos[t] = new_velo.rotate_xy(theta)

            # store controls for step

            test_controls[t] = ControlStep(duration = delta_t, controls = SimpleControllerState(throttle = 1.0, steer = turn_inputs[t]))

            # find and store lagrangian for step

            R = 2189235679380
            r_vec = test_positions[t].__sub__(target)
            kinetic = .5 * 180 * test_velos[t].length() ** 2
            potential = ( .5 * 180 * 1410 ** 2 - kinetic ) + R / (r_vec.length())
            test_lagrangians[t] = kinetic - potential
            
        print(f"test_velos = {test_velos}")
        print(f"Test Ls = {test_lagrangians}")
        idx = np.argmin(test_lagrangians)
        pos_steps[time_step] = test_positions[idx]
        velocity_steps[time_step] = test_velos[idx]
        L_steps[time_step] = test_lagrangians[idx]
        print(f"steer = {test_controls[idx].controls.steer}")
        controls_tracker[time_step-1] = test_controls[idx]

        return find_DSM_path(target, pos_steps, velocity_steps, L_steps,
                             controls_tracker, num_guess_paths, num_time_steps, time_step+1)

    return Sequence(controls_tracker)

def find_turn_radius(v):
    if v == 0:
        return 0
    return 1.0 / curvature(v)

def curvature(v):
    if 0.0 <= v < 500.0:
        return 0.006900 - 5.84e-6 * v
    elif 500.0 <= v < 1000.0:
        return 0.005610 - 3.26e-6 * v
    elif 1000.0 <= v < 1500.0:
        return 0.004300 - 1.95e-6 * v
    elif 1500.0 <= v < 1750.0:
        return 0.003025 - 1.10e-6 * v
    elif 1750.0 <= v < 2500.0:
        return 0.001800 - 0.40e-6 * v
    else:
        return 0.0

def find_throttle_accel(v):
    if 0.0 <= v < 1400.0:
        return 1600 - 1.028571 * v
    elif 1400.0 <= v < 1410.0:
        return 22560 - 16 * v
    else:
        return 0.0