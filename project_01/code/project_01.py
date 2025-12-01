"""
--------------------------------------------------------------------------
Midterm Project: Pomodoro Study Timer
--------------------------------------------------------------------------
License:   
Copyright 2025 Paige Rennie

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------

Use the following hardware components to make a programmable pomodoro timer:  
  - HT16K33 Display
  - 2 x Button
  - Red LED
  - Blue LED
  - White LED
  - Buzzer
  - 4 x Yellow LEDs
  - Potentiometer (analog input)
  - 4 x Resistors

Requirements:
  - Hardware:
    - Before starting: all LEDs are off and screen displays time for Pomodoro
    - When in Pomodoro Timer mode, blue LED is on and time on LED display counts down.
    - Buzzer sounds to start mode.
    - When in short break mode, red LED is on and time on LED display counts down (5 mins).
    - When in long break mode, white LED is on and time on LED display coutns down (10 mins).
    - User Interaction:
        - Blue button pressed to start Pomodoro Timer mode 
        - Press blue button again to end Pomodoro or break mode early 
        - Green button pressed to reset system or restart after final Pomodoro and long break
        - User can rotate potentiometer to change the length of the time of the Pomodoro

Uses:
  - HT16K33 display library developed in class
    - Library updated to add "set_digit_raw()", "set_colon()"

"""
import time

import ht16k33       as HT16K33
import button        as BUTTON
import potentiometer as POT
import led           as LED
import buzzer_music  as MUSIC

# ------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------

POT_DIVIDER        = 128       # Divider used to help reduce potentiometer granularity

# ------------------------------------------------------------------------
# Global variables
# ------------------------------------------------------------------------

# None

# ------------------------------------------------------------------------
# Functions / Classes
# ------------------------------------------------------------------------

class PomodoroTimer():
    """ PomodoroTimer """
    reset_time      = None
    button_blue     = None   
    button_green    = None
    blue_led        = None
    red_led         = None
    white_led       = None
    pomodoro_led_1  = None
    pomodoro_led_2  = None
    pomodoro_led_3  = None
    pomodoro_led_4  = None
    potentiometer   = None
    display         = None
    music           = None
    debug           = None
    
    def __init__(self, reset_time=2.0, button_blue="P2_2", button_green="P2_3",
                       blue_led="P2_4", red_led="P2_6", white_led="P2_8",
                       pomodoro_led_1="P2_27", pomodoro_led_2="P2_29",
                       pomodoro_led_3="P2_31", pomodoro_led_4="P2_33",
                       potentiometer="P1_19", 
                       i2c_bus=1, i2c_address=0x70, buzzer='P2_1', debug=False):
        """ Initialize variables and set up display """

        self.reset_time     = reset_time
        self.button_blue    = BUTTON.Button(button_blue)
        self.button_green   = BUTTON.Button(button_green)
        self.blue_led       = LED.LED(blue_led)
        self.red_led        = LED.LED(red_led)
        self.white_led      = LED.LED(white_led)
        self.pomodoro_led_1 = LED.LED(pomodoro_led_1)
        self.pomodoro_led_2 = LED.LED(pomodoro_led_2)
        self.pomodoro_led_3 = LED.LED(pomodoro_led_3)
        self.pomodoro_led_4 = LED.LED(pomodoro_led_4)
        self.potentiometer  = POT.Potentiometer(potentiometer)
        self.display        = HT16K33.HT16K33(i2c_bus, i2c_address)
        self.music          = MUSIC.BuzzerMusic(buzzer)
        self.debug          = debug
        
        self._setup()
    
    # End def
    
    
    def _all_off(self):
        """Setup the hardware components, turn off all LEDs, clear display and silence buzzer."""

        self.blue_led.off()
        self.red_led.off()
        self.white_led.off()
        self.pomodoro_led_1.off()
        self.pomodoro_led_2.off()
        self.pomodoro_led_3.off()
        self.pomodoro_led_4.off()
        self.display.clear()
        self.display.update(0)
        self.music.buzzer.stop()

    # End def
    
    def _setup(self):
        """Setup the hardware components, turn off all LEDs, clear display and silence buzzer."""
        self._all_off()
    
    # End def
    
    def _set_stage_leds(self, mode):
        """Turns on an led to tell you which mode we are in"""
        self.blue_led.off()
        self.red_led.off()
        self.white_led.off()
        
        if mode == "study":
            self.blue_led.on()
        elif mode == "short_break":
            self.red_led.on()
        elif mode == "long_break":
            self.white_led.on()
            
    # End def
            
    def _update_pomodoro_leds(self,cycle):
        """Turns on an led for each pomodoro cycle completed"""
        leds = [self.pomodoro_led_1,
                self.pomodoro_led_2,
                self.pomodoro_led_3,
                self.pomodoro_led_4]
        for i, led in enumerate(leds,start=1):
            if cycle >= i:
                led.on()
            else:
                led.off()
                
    # End def
                
    def _map_pot_to_minutes(self):
        """Reads the potentiometer and convert to a study time in minutes."""
    
        raw_value = self.potentiometer.get_value()  # 0-4095
        fraction = raw_value/4095.0 # 0.0-1.0
        
        min_time = 15.0                        # minutes
        max_time = 45.0
        
        return min_time + (max_time - min_time) * fraction
        
    # End def
    
    def _show_seconds(self,seconds_left):
        """Displays mm:ss on 7-segment display."""
        minutes = seconds_left // 60
        seconds = seconds_left % 60
        
        # valye like 25:00 --> 2500, 4:30 --> 430, etc
        value = minutes * 100 + seconds
        if value <0:
            value = 0;
        if value > 9999:
            value = 9999
            
        self.display.update(value)
        
    # End def
        
    def _warning_beeps(self,mode,seconds_left):
        """Buzzer warning at the end of each phase."""
        # Gives 1min warning for study phases and 30s warning for short break
        if mode == "study" and seconds_left == 60:
            # one short high sound
            self.music.play_note(MUSIC.NOTE_A5, 0.2, True)
        elif mode == "short_break" and seconds_left == 30: 
            # one short lower sound
            self.music.play_note(MUSIC.NOTE_E5, 0.2, True)
            
    # End def

    # --------------------------------------------------------------------------
    # Main run loop that controls the pomodoro
        
    def run(self):
        """Execute the main program."""
        
        # Green button: power on/off and restarting the cycle
        # Blue button: starts phases
        
        print("Begin Run")
        
        # Waits for first button press before anything starts
        self._all_off()
        self.display.update(0)
        print("Waiting for first press...")
        
        # either blue or green is "on" initially
        while not self.button_blue.is_pressed() or self.button_green.is_pressed():
            study_minutes = self._map_pot_to_minutes()
            self._show_seconds(int(study_minutes * 60))
            time.sleep(0.05)
            
        time.sleep(0.2) 
        
        short_break_min = 5
        long_break_min = 10
        
        mode = "idle"     # options are: idle, pomodoro, short_break, long_break
        cycle = 0
        end_time = None
        
        self._setup()
        
        try:
            while True:
                # Green button instructions 
                if self.button_green.is_pressed():
                    self._all_off()
                    mode = "idle"
                    cycle = 0
                    end_time = None
                    time.sleep(0.25)
                
                # In idle mode, waiting for instruction and potentiometr
                if mode == "idle":
                    # shows the time on display
                    study_minutes = self._map_pot_to_minutes()
                    seconds_preview = int(study_minutes*60)
                    self._show_seconds(seconds_preview)
                    
                    # press of blue button starts the first pomodoro
                    if self.button_blue.is_pressed():
                        mode = "study"
                        end_time = time.time() + study_minutes*60
                        self._set_stage_leds(mode)
                        self._update_pomodoro_leds(cycle)
                        self.music.play_note(MUSIC.NOTE_A4, 0.15, True)
                        time.sleep(0.25)
                
                # study or break modes        
                else:
                    remaining = int(end_time - time.time())
                    if remaining < 0:
                        remaining = 0
                        
                    self._show_seconds(remaining)
                    self._warning_beeps(mode, remaining)
                    
                    # blue button pressed whilst running takes you to the next phase
                    if self.button_blue.is_pressed():
                        remaining = 0 # skips to the next phase if that part of the cycle hasn't ended yet
                        time.sleep(0.25)
                    
                    if remaining == 0:

                        if mode == "study":
                            # Study mode just finished
                            # Beep pattern
                            self.music.play_note(MUSIC.NOTE_C6, 0.2, True)
                            self.music.play_note(MUSIC.NOTE_C6, 0.2, True)
                    
                            # Completed one pomodoro
                            cycle += 1
                    
                            # Determine break type
                            if cycle % 4 == 0:
                                mode = "long_break"
                                end_time = time.time() + long_break_min * 60
                            else:
                                mode = "short_break"
                                end_time = time.time() + short_break_min * 60
                    
                        else:
                            # Break mode just finished
                            self.music.play_note(MUSIC.NOTE_G5, 0.3, True)
                    
                            if cycle >= 4:
                                # Completed all 4 pomodoros → reset
                                mode = "idle"
                                cycle = 0
                                end_time = None
                                self._set_stage_leds("idle")
                            else:
                                # Start next study block
                                study_minutes = self._map_pot_to_minutes()
                                mode = "study"
                                end_time = time.time() + study_minutes * 60

                    
                self._set_stage_leds(mode)
                self._update_pomodoro_leds(cycle)
                
                time.sleep(0.05) # how often the main loop repeats
        
        except KeyboardInterrupt: 
            self._all_off()
                    
                        
        print("all off")
        
    # End def
                
    def cleanup(self):
        """Turn everything off and clean up hardware."""
        self._all_off()

        # Cleaning up each component
        self.blue_led.cleanup()
        self.red_led.cleanup()
        self.white_led.cleanup()
        self.pomodoro_led_1.cleanup()
        self.pomodoro_led_2.cleanup()
        self.pomodoro_led_3.cleanup()
        self.pomodoro_led_4.cleanup()
        self.potentiometer.cleanup()
        self.music.cleanup()
        
    # End def

# ------------------------------------------------------------------------
# Main script
# ------------------------------------------------------------------------

if __name__ == "__main__":
    
    print("Program Start")
    timer = PomodoroTimer()
    timer.run()
    
    print("Program Complete")
    

