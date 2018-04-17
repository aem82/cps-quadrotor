
from numpy import ones,vstack
from numpy.linalg import lstsq
import math


def getLine(x_i, y_i, x_f, y_f):
    points = [(x_i,y_i),(x_f,y_f)]
    x_coords, y_coords = zip(*points)
    A = vstack([x_coords,ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords)[0]
    #print("Line Solution is y = {m}x + {c}".format(m=m,c=c))
    return m, c

def lineDistance(a, c, x, y, b=-1):
    return abs(a*x + b*y + c)/math.sqrt(math.pow(x,2) +math.pow(b,2))


POSITION_COMMAND = "POSITION COMMAND"
T_POS = 0
X_POS = 1
Y_POS = 2
Z_POS = 3

COLS = "t(s)"
TIME = "TIME"
OUT_FILE = "CPS_data.txt"
IN_FILE = "CPS.txt"
distTimeErrors = []


x_i = 0
y_i = 0
z_i = 0
x_f = 0
y_f = 0
z_f = 0

m = 0
c = 0


newList = []
isCommandCoord = False
isInitialCoord = False
for line in open(IN_FILE):
    if line.split() == None or len(line.split())== 0:
        continue

    if COLS in line:
        isInitialCoord = True
        continue

    if isCommandCoord:
        isCommandCoord = False
        x_f = float(line.split()[X_POS])
        y_f = float(line.split()[Y_POS])
        #z_f = line.split()[3]

    if POSITION_COMMAND in line:
        if len(newList)>0:
            distTimeErrors.append(newList)
        newList= []
        isCommandCoord = True

    else:
        if isInitialCoord:
            isInitialCoord = False
            t = line.split()[0]
            x_i = float(line.split()[X_POS])
            y_i = float(line.split()[Y_POS])
            #z_i = line.split()[3]
            m, c = getLine(x_i, y_i, x_f, y_f)

        else:
            t = float(line.split()[T_POS])
            x = float(line.split()[X_POS])
            y = float(line.split()[Y_POS])
            dist = 0
            dist = lineDistance(m, c, x, y)
            newList.append([t, (x, y), dist])
distTimeErrors.append(newList)
dataFile = open(OUT_FILE, "w")
i = 1
for trial in distTimeErrors:
    dataFile.write("\n"+"Trial: "+str(i)+"\n")
    dataFile.write("time  point  error"+"\n")
    for line in trial:
        dataFile.write(str(line))
        dataFile.write("\n")
    i+=1


