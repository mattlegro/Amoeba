def disp_debug_text(self,car_velocity):

# Draw useful debugging text

    debug_text = 'I am a turd bot.\n'
    debug_text += f'Speed: {car_velocity.length():.1f}\n'
    debug_text += 'Current Action: '

    if debug_text:
                                   
        text_y = 500 - debug_text.count('\n')
        self.renderer.draw_string_2d(10,text_y,1,1,debug_text,self.renderer.white())

    return