from AHRS import AHRS
import time
navx = AHRS()


while True:
    #pass
    time.sleep(1)
    print("Test: {}".format(navx.getYaw()))
   #print("Yaw: {}, Pitch: {}, Roll: {}".format(navx.getYaw(), navx.getPitch(), navx.getRoll()))
