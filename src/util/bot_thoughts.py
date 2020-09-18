       
def disp_bot_thoughts(self,ball_prediction,target,car_location,target_future):

    # Get Ball Prediction Line
        
    ball_path = [0] * 300

    for s in range(300):
            
        slice_location = ball_prediction.slices[s]
        ball_path[s] = slice_location.physics.location        

    # Draw some things to help understand what the bot is thinking
    self.renderer.draw_polyline_3d(ball_path, self.renderer.blue())
    self.renderer.draw_polyline_3d([car_location, target_future, target], self.renderer.white())
    self.renderer.draw_rect_3d(target_future, 6, 6, True, self.renderer.cyan(), centered=True)

    return