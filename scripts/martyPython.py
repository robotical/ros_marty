import time
import socket
import struct

# Robot Network Info
DEF_HOST = 'localhost'
DEF_PORT = 1569

cmds = {
  "hello": 1,
  "movejoint": 2,
  "lean": 3,
  "walk": 4,
  "eyes": 5,
  "kick": 6,
  "liftleg": 7,
  "lowerleg": 8,
  "celebrate": 9,
  "dance": 10,
  "rollerskate": 11,
  "arms": 12,
  "demo": 13,
  "stop": 14
}

# DEFAULT POSES
EYES_ANGRY = 80
EYES_NORMAL = 60
EYES_EXCITED = 20
EYES_WIDE = -100

# COMMUNICATION DEFINITIONS
CMD_LEFT = 0
CMD_RIGHT = 1
CMD_FORW = 3
CMD_BACK = 4

J_HIP = 0
J_TWIST = 1
J_KNEE = 2
J_LEG = 3

# Socket Methods
def init_socket(host, port=DEF_PORT):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (host, port)
    print "Connecting to %s port: %s" % server_address
    sock.connect(server_address)
    return sock

def sendCmdSock(sock, cmd, data):
    print cmd
    sendlist = struct.pack('<i', cmd)
    for x in xrange(0,len(data)):
        sendlist = sendlist + struct.pack('<i', data[x])
    sock.sendall(sendlist)
    rec_msg = sock.recv(4096)
    if rec_msg != "Success":
        print "Warning: Cmd Failed = " + rec_msg
    sock.close()

def sendCmd(cmd, data, host=DEF_HOST):
    sock = init_socket(host, DEF_PORT)
    sendCmdSock(sock, cmd, data)
    time.sleep(0.02)    # ROS spins at 50Hz

# Marty Commands
def arm(side, amount = 0):
    if (side == "left"):
        sendCmd(cmds["arms"], [CMD_LEFT, amount])
    elif (side == "right"):
        sendCmd(cmds["arms"], [CMD_RIGHT, amount])
    else:
        print "Error: Please select which arm to move (left or right)"

def arms(rarm = 0, larm = 0):
    sendCmd(cmds["arms"], [rarm, larm])

def celebrate():
    return sendCmd(cmds["celebrate"],[])

def dance(robot_name, robot_id = 0):
    sendCmd(cmds["dance"], [robot_id], robot_name)

def demo():
    sendCmd(cmds["demo"], [])

def eyes(amount = 0, amount2 = 0):
    sendCmd(cmds["eyes"], [amount, amount2])

def hello():
    sendCmd(cmds["hello"], [])

def kick(leg = "left", movetime = 3000):
    if (leg == "left"):
        sendCmd(cmds["kick"], [CMD_LEFT, movetime])
    if (leg == "right"):
        sendCmd(cmds["kick"], [CMD_RIGHT, movetime])

def kickLeft(movetime = 3000):
    kick("left", movetime)

def kickRight(movetime = 3000):
    kick("right", movetime)

##
## @brief      Marty leans in any direction
##
## @param      direction  Select left, right, forward or backward
## @param      amount     Positive or negative, between 100 and -100
## @param      movetime   In milliseconds, recommended 2000
##
## @return     Returns False if wrong direction is set
##
def lean(direction, amount = 100, movetime = 2000):
    if direction == "left":
        sendCmd(cmds["lean"], [CMD_LEFT, min(abs(amount), 100), movetime])
    elif direction == "right":
        sendCmd(cmds["lean"], [CMD_RIGHT, min(abs(amount), 100), movetime])
    elif direction == "forward":
        sendCmd(cmds["lean"], [CMD_FORW, min(abs(amount), 100), movetime])
    elif direction == "backward":
        sendCmd(cmds["lean"], [CMD_BACK, min(abs(amount), 100), movetime])
    else:
        print "Error: Please write the lean direction (left, right, forward, backward)"
        return False

def leanLeft(amount = 100, movetime = 2000):
    lean("left", amount, movetime)

def leanRight(amount = 100, movetime = 2000):
    lean("right", amount, movetime)

def leanForward(amount = 100, movetime = 2000):
    lean("forward", amount, movetime)

def leanBackward(amount = 100, movetime = 2000):
    lean("backward", amount, movetime)

def liftLeg(leg, amount = 100, movetime = 2000):
    if leg == "left":
        sendCmd(cmds["liftleg"], [CMD_LEFT, min(abs(amount), 100), movetime])
    elif leg == "right":
        sendCmd(cmds["liftleg"], [CMD_RIGHT, min(abs(amount), 100), movetime])
    else:
        print "Error: Please write the leg to lift (left or right)"
        return False

def liftLeftLeg(amount = 100, movetime = 2000):
    return liftLeg("left", amount, movetime)

def liftRightLeg(amount = 100, movetime = 2000):
    return liftLeg("right", amount, movetime)

def lowerLeg(movetime = 1000):
    return sendCmd(cmds["lowerleg"], [movetime])

def moveJoint(side, joint, amount, movetime = 2000):
    if joint == "hip":
        if side == "left":
            sendCmd(cmds["movejoint"], [CMD_LEFT, J_HIP, amount, movetime])
        elif side == "right":
            sendCmd(cmds["movejoint"], [CMD_RIGHT, J_HIP, amount, movetime])
        else:
            print "Error: Please select left or right joint"
            return False
    elif joint == "twist":
        if side == "left":
            sendCmd(cmds["movejoint"], [CMD_LEFT, J_TWIST, amount, movetime])
        elif side == "right":
            sendCmd(cmds["movejoint"], [CMD_RIGHT, J_TWIST, amount, movetime])
        else:
            print "Error: Please select left or right joint"
            return False
    elif joint == "knee":
        if side == "left":
            sendCmd(cmds["movejoint"], [CMD_LEFT, J_KNEE, amount, movetime])
        elif side == "right":
            sendCmd(cmds["movejoint"], [CMD_RIGHT, J_KNEE, amount, movetime])
        else:
            print "Error: Please select left or right joint"
            return False
    elif joint == "leg":
        if side == "left":
            sendCmd(cmds["movejoint"], [CMD_LEFT, J_LEG, amount, movetime])
        elif side == "right":
            sendCmd(cmds["movejoint"], [CMD_RIGHT, J_LEG, amount, movetime])
        else:
            print "Error: Please select left or right joint"
            return False
    else:
        print "Error: Please select joint: hip, twist, knee or leg"
        return False

def moveHip(leg, amount = 100, movetime = 2000):
    if leg == "left":
        moveJoint("left", "hip", amount, movetime)
    elif leg == "right":
        moveJoint("right", "hip", amount, movetime)
    else:
        print "Error: Please select left or right hip"
        return False

def moveLeftHip(amount = 100, movetime = 2000):
    moveHip("left", amount, movetime)

def moveRightHip(amount = 100, movetime = 2000):
    moveHip("right", amount, movetime)

def moveLeg(side, amount = 100, movetime = 2000):
    if side == "left":
        moveJoint("left", "leg", amount, movetime)
    elif side == "right":
        moveJoint("right", "leg", amount, movetime)
    else:
        print "Error: Please select left or right leg"
        return False

def moveLeftLeg(amount = 100, movetime = 2000):
    moveLeg("left", amount, movetime)

def moveRightLeg(amount = 100, movetime = 2000):
    moveLeg("right", amount, movetime)

def moveRightLegForward(amount = 100, movetime = 2000):
    return moveRightLeg(-amount, movetime)

def moveRightLegBackward(amount = 100, movetime = 2000):
    return moveRightLeg(amount, movetime)

def moveLeftLegForward(amount = 100, movetime = 2000):
    return moveLeftLeg(-amount, movetime)

def moveLeftLegBackward(amount = 100, movetime = 2000):
    return moveLeftLeg(amount, movetime)

def rollerSkate():
    sendCmd(cmds["rollerskate"], [])

def standStraight(movetime = 1000):
    leanForward(0, movetime)
    leanLeft(0, movetime)

def stop():
    sendCmd(cmds["stop"], [])

def walk(numsteps = 2, turn = 0, movetime = 3000, stepLength = 50):
    sendCmd(cmds["walk"], [numsteps, turn, movetime, stepLength])
