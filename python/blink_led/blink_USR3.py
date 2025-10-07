# -*- coding: utf-8 -*-
"""
--------------------------------------------------------------------------
LED Blinker
--------------------------------------------------------------------------
License:   
Copyright 2025 - Paige Rennie

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------

# Note 5Hz = 0.2 secs

#import pytest
import os
import errno

# Set the 3rd LED as the one to blink
led = "USR3"

# 5Hz or 5 full on/off cycles means each cycle is 0.2s and the LED is on for 0.1s
DELAY = 0.1

import Adafruit_BBIO.GPIO as GPIO

def teardown_module(module):
    GPIO.cleanup()

class TestLED:
    def set_brightness(self, state, led, name):
        GPIO.setup(led, GPIO.OUT)
        GPIO.output(led, state)
        prefix = "/sys/class/leds/beaglebone:green:{0}/brightness"
        path = prefix.format(led.lower())
        value = self.read_led_file(path)
        if value == "":
            path = prefix.format(name)
            value = self.read_led_file(path)
        if state == 1:
            assert int(value) > 0
        else:
            assert int(value) == 0

    def read_led_file(self, path):
        try:
            return open(path).read()
        except (IOError, e):
            if e.errno == errno.ENOENT:
                return ""

    def set_all_leds(self, state):
        test_leds = { "USR0": "heartbeat", \
                      "USR1": "mmc0", \
                      "USR2": "cpu0", \
                      "USR3": "mmc1" }
        for led, name in test_leds.items():
            self.set_brightness(state, led, name)
            GPIO.cleanup()
   
    def test_brightness_high(self):
        self.set_all_leds(GPIO.HIGH)

    def test_brightness_low(self):
        self.set_all_leds(GPIO.LOW)

