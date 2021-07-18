import sys
import math
import numpy as np

#global variables
loop = 0
speed = 0
boostAvailable = True
checkpoints = []
path = []
lastDist = 10**10
maxCoordsAndDist = (0,0,0)
firstLapOver = False
checkpointPassed = False
nextCheckpoint = (0,0)
hadSpeedNegativeValue = False
myAngleToX = 0
checkpointAngleToX = 0
myAngleToY = 0
checkpointAngleToY = 0
#functions

def calcAngleBetween(x,y,oppX,oppY,nextX,nextY):
    v1 = (nextX-x,nextY-y)
    v2 = (nextX-oppX,nextY-oppY)
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    return round(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)),3)
def calcDistanceBetween(x,y,xp,yp):
    return ((x-xp)**2 + (y-yp)**2)**0.5
def calcThrust(dist,angle,nextX,nextY,x,y,oppX,oppY):
    thrust=100
    maxX,maxY,maxDist = maxCoordsAndDist
    global boostAvailable
    nextDestinationX, nextDestinationY = nextCheckpoint
    distanceToNextCheckpoint = calcDistanceBetween(nextX,nextY,nextDestinationX,nextDestinationY)
    if loop==0:
        thrust = 'BOOST'
        boostAvailable = False
    elif dist < 3500 and speed > 550:
        thrust = 0
    elif firstLapOver and dist<3000 and distanceToNextCheckpoint<3000 and not hadSpeedNegativeValue:
        thrust = 30
    elif 1400<dist<2000 and not hadSpeedNegativeValue:
        thrust = int(100**(dist/2000))
    elif angle>10:
        thrust = int(100*max((1-(angle/180)),0))
    elif boostAvailable and (calcDistanceBetween(x,y,nextX,nextY)>calcDistanceBetween(oppX,oppY,nextX,nextY) and calcAngleBetween(x,y,oppX,oppY,nextX,nextY)>0.5) and angle<10 and nextX == maxX and nextY == maxY and firstLapOver:#age jolotar boodam zavie mohem ni
        thrust = 'BOOST'
        boostAvailable = False#too akharin checkpoint bayad boosto bezanam
    return thrust
def hitOpponent(x,y,oppX,oppY):
    if calcDistanceBetween(x,y,oppX,oppY)<1000 and calcDistanceBetween(x,y,nextX,nextY)>calcDistanceBetween(oppX,oppY,nextX,nextY):
        result = True
    else:
        result = False
    return result
def checkpointsAnalyzer(x,y):
    p = (x,y)
    global checkpoints
    global path
    global maxCoordsAndDist
    global firstLapOver
    global checkpointPassed 
    global nextCheckpoint
    checkpointPassed = False
    if p not in checkpoints:
        checkpoints.append(p)
    if len(path) == 0:
        path.append(p)
    elif path[-1] != p:
        checkpointPassed = True
        if p in path:
            firstLapOver = True
        path.append(p)
    if firstLapOver:
        checkpointIndex = checkpoints.index((nextX,nextY))
        nextCheckpointIndex = checkpointIndex+1 if checkpointIndex+1 != len(checkpoints) else 0
        destinationX, destinationY = checkpoints[checkpointIndex]
        nextCheckpoint = checkpoints[nextCheckpointIndex]
    if len(path) > 1:
        lastMaxDist = maxCoordsAndDist[2]
        distToCheckpoint = calcDistanceBetween(x,y,path[-2][0],path[-2][1])
        if distToCheckpoint > lastMaxDist:
            maxCoordsAndDist = (x,y,distToCheckpoint)
def storeLastDist(dist): 
    global lastDist
    lastDist = dist
def steer(x,y,nextX,nextY,dist):
    x=y=chX=chY=w=h=0
    if len(path)>1:
        x,y=path[-2]
        chX,chY=path[-1]
    else:
        return path[0]
    
    if hadSpeedNegativeValue:
        return (nextX,nextY)

    if firstLapOver and dist<1500:
        nextDestinationX, nextDestinationY = nextCheckpoint
        return (nextDestinationX,nextDestinationY)
    else:
        if x-chX >= 0:
            w = chX + 400
        else:
            w = chX - 400
        if y-chY >= 0: 
            h = chY + 400
        else:
            h = chY - 400
    return (w,h)
def calcSpeed(dist):
    global speed
    global hadSpeedNegativeValue
    speed = 0
    speed = lastDist-dist
    if speed < 0 and not hadSpeedNegativeValue and not checkpointPassed and dist<2000:
        hadSpeedNegativeValue=True
    elif checkpointPassed:
        hadSpeedNegativeValue=False
def calcAngleToZero(x,y,nextX,nextY):
    global myAngleToX
    global checkpointAngleToX
    global myAngleToY
    global checkpointAngleToY
    myAngleToX = calcAngleBetween(x,y,100,0,0,0)
    checkpointAngleToX = calcAngleBetween(nextX,nextY,100,0,0,0)
    myAngleToY = calcAngleBetween(x,y,0,100,0,0)
    checkpointAngleToY = calcAngleBetween(nextX,nextY,0,100,0,0)


while True:
    x, y, nextX, nextY, dist, angle = [int(i) for i in input().split()]
    absAngle = abs(angle)
    oppX, oppY = [int(i) for i in input().split()]
    checkpointsAnalyzer(nextX,nextY)
    calcSpeed(dist)
    calcAngleToZero(x, y, nextX, nextY)
    thrust = calcThrust(dist,absAngle,nextX,nextY,x,y,oppX,oppY)
    goToX, goToY = steer(x,y,nextX,nextY,dist)

    # print('dist,lastDist=>'+str(dist)+','+str(lastDist), file=sys.stderr, flush=True)
    # print('speed=>'+str(speed), file=sys.stderr, flush=True)
    # print('X,Y=>'+str(x)+','+str(y), file=sys.stderr, flush=True)
    # print('nextX,nextY=>'+str(nextX)+','+str(nextY), file=sys.stderr, flush=True)
    # print('goToX,goToY=>'+str(goToX)+','+str(goToY), file=sys.stderr, flush=True)
    # print('angle=>'+str(angle), file=sys.stderr, flush=True)
    # print('thrust=>'+str(thrust), file=sys.stderr, flush=True)
    # print('path=>'+str(path), file=sys.stderr, flush=True)
    # print('checkpointPassed=>'+str(checkpointPassed), file=sys.stderr, flush=True)
    # print('firstLapOver=>'+str(firstLapOver), file=sys.stderr, flush=True)
    # print('maxCoordsAndDist=>'+str(maxCoordsAndDist), file=sys.stderr, flush=True)
    # print('calcDist=>'+str(calcDistanceBetween(x,y,oppX,oppY)), file=sys.stderr, flush=True)
    # print('hitOpponent=>'+str(hitOpponent(x,y,oppX,oppY)), file=sys.stderr, flush=True)
    # print('AngleBetween=>'+str(calcAngleBetween(x,y,oppX,oppY,nextX,nextY)), file=sys.stderr, flush=True)
    # print('nextCheckpoint=>'+str(nextCheckpoint), file=sys.stderr, flush=True)
    # print('angleX=>'+f'{str(myAngleToX)},{str(checkpointAngleToX)}', file=sys.stderr, flush=True)
    # print('angleY=>'+f'{str(myAngleToY)},{str(checkpointAngleToY)}', file=sys.stderr, flush=True)
    # print('hadSpeedNegativeValue=>'+str(hadSpeedNegativeValue), file=sys.stderr, flush=True)
    
    print(f'{goToX} {goToY} {thrust}')
    storeLastDist(dist)
    loop+=1