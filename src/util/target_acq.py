from util.vec import Vec3
from util.ball_prediction_analysis import find_slice_at_time
import math

def  time_to_target_estimate(target_current,target_velocity,car_location,car_velocity) -> float:


    relative_location = target_current.__sub__(car_location)
    relative_xy = relative_location.flat()
    c = relative_xy.length()
    target_vxy = target_velocity.flat()
    car_vxy = car_velocity.flat()
    if car_vxy.length == 0:
        car_vxy.x = .1
        car_vxy.y = .1
        
    #print(f"Target Velocity = {target_vxy}")
    #print(f"Car Velocity = {car_vxy}")
    #print(f"Relative Location = {relative_location}")
    #print(f"Relative Distance = {c}")
    A = math.acos(target_vxy.dot(relative_xy)/target_vxy.length()/c)
    #print(f"Target_Velocity Magnitude = {target_vxy.length()}")
    #print(f"Angle A in Radians = {A}")
    #print(f"Car Speed = {car_vxy.length()}")
    
    try:
        # This means car can actually intersect target path
        B = math.asin(target_vxy.length()**math.sin(A)/car_vxy.length())
        #print(f"Angle B in Radians = {B}")

    except ArithmeticError as error:
        # car cannot reach path at current speed
        time_to_target = 6
        
    else:
        C = math.pi - A - B
        #print(f"Angle C in Radians = {C}")
        b = math.sin(B)*c/math.sin(C)
        #print(f"Distance to Intercept b = {b}")
        time_to_target = b / target_vxy.length()
        #print(f"Calculated time to Intercept = {time_to_target}")
        if time_to_target > 6:
            time_to_target = 6
    finally:
        return time_to_target
     