from typing import List
import math

from rlbot.utils.structures.game_data_struct import GameTickPacket, FieldInfoPacket, PlayerInfo

from util.vec import Vec3
from util.orientation import Orientation


class MyCarInfo:

    def __init__(self):
        self.my_car: float
        self.location: Vec3 
        self.velocity: Vec3 
        self.rotation: Rotator 
        self.angular_V: Vec3 
        self.orientation: Orientation

    def find_car(self, index):
        self.my_car: my_car = index

    def get_info(self,packet):

        self.location: location = Vec3(packet.game_cars[self.my_car].physics.location)
        self.velocity: velocity = Vec3(packet.game_cars[self.my_car].physics.velocity)
        self.rotation: rotation = packet.game_cars[self.my_car].physics.rotation
        self.angular_V: angular_V = Vec3(packet.game_cars[self.my_car].physics.angular_velocity)
        self.orientation: orientation = Orientation(self.rotation)
