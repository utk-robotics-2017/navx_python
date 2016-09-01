from AHRS import AHRS

navx = AHRS()

navx.start()

while True:
    pass
    #print "Yaw: %f, Pitch: %f, Roll: %f" % (navx.getYaw(), navx.getPitch(), navx.getRoll())
