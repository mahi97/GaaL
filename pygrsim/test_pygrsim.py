from pygrsim import *

ssl = SSLWorld(0.016)
ssl.reset()
done = False

action_0 = []
action_1 = []

for a in range(6):
    act = Action()