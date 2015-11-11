#!/usr/bin/env python
# -*-coding:utf-8 -*
  
try:
    import RPi.GPIO as GPIO
except RuntimeError:
    print("Error importing RPi.GPIO!  This is probably because you need superuser privileges.  You can achieve this by using 'sudo' to run your script")

GPIO.setmode(GPIO.BCM)  #use native pin numbers
#GPIO.setwarnings(False)
  
# GPIO 14, 15 ,18 and 23 pulled up as inputs of the encoders
# they detect both falling and rising edges
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#other possibilities : 
#GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
#GPIO.setup(channel, GPIO.OUT, initial=GPIO.HIGH)

  
# now we'll define the threaded callback function  
# this will run in another thread when our event is detected  
def encoder_1(channel):  
    print "Rising edge detected on port 24 - even though, in the main thread,"
    print "we are still waiting for a falling edge - how cool?\n"

def encoder_2(channel):  
    print "Rising edge detected on port 24 - even though, in the main thread,"
    print "we are still waiting for a falling edge - how cool?\n"
  
print "Make sure you have a button connected so that when pressed"  
print "it will connect GPIO port 23 (pin 16) to GND (pin 6)\n" 
print "You will also need a second button connected so that when pressed"  
print "it will connect GPIO port 24 (pin 18) to 3V3 (pin 1)"  
raw_input("Press Enter when ready\n>")
  
# The GPIO.add_event_detect() line below set things up so that  
# when a rising edge is detected on port 24, regardless of whatever   
# else is happening in the program, the function "my_callback" will be run  
# It will happen even while the program is waiting for  
# a falling edge on the other button.  
GPIO.add_event_detect(14, GPIO.BOTH, callback=encoder_1)
GPIO.add_event_detect(15, GPIO.BOTH, callback=encoder_1)

GPIO.add_event_detect(18, GPIO.BOTH, callback=encoder_2)
GPIO.add_event_detect(23, GPIO.BOTH, callback=encoder_2)
 
GPIO.cleanup()           # clean up GPIO on normal exit  
