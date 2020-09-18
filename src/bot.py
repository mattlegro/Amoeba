from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.my_car import MyCarInfo
from util.ball_info import BallInfo
from util.DSM import NestedArrayStructure, find_DSM_path
from util.drive import steer_toward_target
from util.sequence import Sequence, ControlStep
from util.vec import Vec3
from util.bot_thoughts import disp_bot_thoughts
from util.debug_text import disp_debug_text
from util.target_acq import time_to_target_estimate

class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.car_tracker = MyCarInfo()
        self.ball_tracker = BallInfo()

    def initialize_agent(self):
        # Set up information trackers now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())
        self.car_tracker.find_car(self.index)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car. Test Changes.
        """

        # Keep game info up to date
        self.boost_pad_tracker.update_boost_status(packet)
        self.car_tracker.get_info(packet)
        self.ball_tracker.get_info(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            #disp_bot_thoughts(self,ball_prediction,ball_location,car_location,target_location)
            #disp_debug_text(self,car_velocity)
            if controls is not None:
                return controls

        ball_location = Vec3(self.ball_tracker.location)
        ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc

        # Find approximate target based on distance from the ball and its speed
        # Guess based on intersection of straight line x,y distance and cars max travel
        # speed in that direction, maybe adding time to guess if turning is required
        
        # Target Ball

        target = ball_location
        target_velocity = self.ball_tracker.velocity

        # Estimate Intersection Path

        if target_velocity.length() == 0:
            target_future = target
        else:
            time = time_to_target_estimate(target,target_velocity,self.car_tracker.location,self.car_tracker.velocity)
            slice = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + time)
            target_future = Vec3(slice.physics.location)

        # TRY DSM METHOD


        return self.take_DSM_path(packet, target_future)

        # FAILURE

        controls = SimpleControllerState()
        controls.steer = steer_toward_target(self.car_tracker, target_future)
        controls.throttle = 1.0

        disp_bot_thoughts(self,ball_prediction,target,self.car_tracker.location,target_future)
        disp_debug_text(self,self.car_tracker.velocity)

        return controls

    def take_DSM_path(self, packet, target_location):

        # Send some quickchat just for fun
        #self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # generate useful structures for path finder

        num_time_steps = 5
        num_guess_paths = 11
        #L_steps = NestedArrayStructure(num_time_steps,num_guess_paths)
        L_steps = [0] * (num_time_steps + 1)
        velocity_steps, position_steps = L_steps[:], L_steps[:]
        controls_tracker = L_steps[1:]
        velocity_steps[0] = self.car_tracker.velocity
        position_steps[0] = self.car_tracker.location
        
        R = 2189235679380
        r_vec = self.car_tracker.location.__sub__(target_location)
        kinetic = .5 * 180 * self.car_tracker.velocity.length() ** 2
        potential = ( .5 * 180 * 1410 ** 2 - kinetic ) + R / (r_vec.length())
        L_steps[0] = kinetic - potential

        # find controls sequence using DSM method
        self.active_sequence = find_DSM_path(target_location, 
                                             position_steps, velocity_steps, L_steps, 
                                             controls_tracker, num_guess_paths, num_time_steps, 1)

        return self.active_sequence.tick(packet)

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)
