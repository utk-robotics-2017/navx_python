from NavX import get_navx
import time

with get_navx(10) as navx:
    time.sleep(1)

    while True:
        #pass
        time.sleep(1)
        #print("Test: {}".format(navx.getYaw()))
        print("Yaw: {}, Pitch: {}, Roll: {}".format(navx.getYaw(), navx.getPitch(), navx.getRoll()))
