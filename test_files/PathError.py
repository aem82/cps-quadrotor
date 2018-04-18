import numpy
from numpy import ones,vstack
from numpy.linalg import lstsq
import math
import matplotlib
from matplotlib import pyplot

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
#IN_FILE = "CPS.txt"
files = ["1_2018-4-17-13-55-50.txt", "1_2018-4-17-13-58-2.txt","1_2018-4-17-14-32-44.txt","2_2018-4-17-14-1-1.txt","5_2018-4-17-14-3-18.txt", "10_2018-4-17-14-24-58.txt"]
distTimeErrors = []


x_i = 0
y_i = 0
z_i = 0
x_f = 0
y_f = 0
z_f = 0

m = 0
c = 0

for file in files:
    newList = []
    isFinalCoord = False
    isInitialCoord = True
    for line in open(file):
        if line.split() == None or len(line.split())== 0:
            distTimeErrors.append(newList)
            isInitialCoord = True
            continue

        # if COLS in line:
        #     isInitialCoord = True
        #     continue

        if isFinalCoord:
            isFinalCoord = False
            x_f = float(line.split()[X_POS])
            y_f = float(line.split()[Y_POS])
            z_f = line.split()[3]

        # if POSITION_COMMAND in line:
        #     if len(newList)>0:
        #         distTimeErrors.append(newList)
        #     newList= []
        #     isCommandCoord = True
        #
        # else:
        if isInitialCoord:
            isInitialCoord = False
            isFinalCoord=True
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

#plotting
errors = []
maxError = -1
for i in distTimeErrors:
    for j in i:
        errors.append(j[2])
        if j[2]>maxError:
            maxError = j[2]
        #print(str(j[2]))
    #     count+=1
    #     if count>10:
    #         break
    # break

print("Max Error:",maxError)
pyplot.hist(errors, bins=50)
pyplot.xlabel('Distance from straight path (m)')
pyplot.ylabel('Number of Occurances')
pyplot.title('Histogram of Path Error')
pyplot.grid(True)
pyplot.show()

