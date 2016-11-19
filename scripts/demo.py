import time
import math
import martyPython as marty

## Get Marty ready
# marty.hello()

## Walking
NUM_STEPS = 2
TURN = 0      # 0 = Straight, >0 = Right, <0 = Left
WALK_TIME = 3000
STEP_LENGTH = 50
# marty.walk(NUM_STEPS, TURN, WALK_TIME, STEP_LENGTH)

## Kicking
KICK_TIME = 2000
# marty.kickLeft()
# marty.kickRight()
# marty.kick("left", KICK_TIME)

## Arms
RIGHT_ARM = 50
LEFT_ARM = -50
# marty.arms(RIGHT_ARM, LEFT_ARM)

## Eyes
# marty.eyes(marty.EYES_WIDE)

## Leaning
LEAN_AMOUNT = 50
LEAN_TIME = 2000
# marty.leanForward()
# marty.leanBackward()
# marty.lean("left", LEAN_AMOUNT)
# marty.lean("right", LEAN_AMOUNT, LEAN_TIME)
# marty.standStraight()

## Joint Control
MOVE_AMOUNT = 50
MOVE_TIME = 1000
# marty.moveJoint("right", "knee", 40)
# marty.liftLeftLeg()
# marty.moveJoint("left", "twist", MOVE_AMOUNT)
# marty.moveJoint("left", "twist", 0, MOVE_TIME)
# marty.lowerLeg()
# marty.standStraight()

## Example Loop
# for x in range(0,20):
#     marty.eyes(marty.EYES_EXCITED)
#     marty.arms(100,0)
#     time.sleep(0.1)
#     marty.eyes(marty.EYES_NORMAL)
#     marty.arms(0,100)
#     time.sleep(0.1)
# marty.arms()

## Celebrate!
#marty.celebrate()

## Example Step function
# STEP_TIME = 1000
# def step():
#     marty.standStraight()
#     marty.leanRight(60, STEP_TIME)
#     marty.liftLeftLeg(50, STEP_TIME)
#     marty.moveLeftLegForward(50, STEP_TIME)
#     marty.lowerLeg()
#     marty.standStraight()
#     marty.leanLeft(60, STEP_TIME)
#     marty.liftRightLeg(50, STEP_TIME)
#     marty.moveRightLegForward(50, STEP_TIME)
#     marty.lowerLeg()
#     marty.standStraight()
#
# for s in xrange(1,2):
#     step()

## Stop Marty, put this at the end of your script!
marty.stop()
