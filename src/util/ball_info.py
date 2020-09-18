from typing import List

from rlbot.utils.structures.game_data_struct import GameTickPacket, FieldInfoPacket, PlayerInfo

from util.vec import Vec3
from util.orientation import Orientation


class BallInfo:

    def __init__(self):
        self.location: Vec3 
        self.velocity: Vec3 
        self.rotation: Rotator 
        self.angular_V: Vec3 

    def get_info(self,packet):

        self.location: location = Vec3(packet.game_ball.physics.location)
        self.velocity: velocity = Vec3(packet.game_ball.physics.velocity)
        self.rotation: rotation = packet.game_ball.physics.rotation
        self.angular_V: angular_V = Vec3(packet.game_ball.physics.angular_velocity)

