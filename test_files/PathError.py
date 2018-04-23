import numpy
from numpy import ones,vstack
from numpy.linalg import lstsq
import math
import matplotlib
import os
from matplotlib import pyplot

def getLine(x_i, y_i, x_f, y_f):
    points = [(x_i,y_i),(x_f,y_f)]
    x_coords, y_coords = zip(*points)
    A = vstack([x_coords,ones(len(x_coords))]).T
    m, c = lstsq(A, y_coords)[0]
    #print("Line Solution is y = {m}x + {c}".format(m=m,c=c))
    return m, c

def lineDistance(a, c, x, y, b=-1):
    return abs(a*x + b*y + c)/math.sqrt(math.pow(a,2) +math.pow(b,2))

def pointDistance(a, b, x, y):
    return math.sqrt(math.pow((a-x),2)+math.pow((b-y),2))

POSITION_COMMAND = "POSITION COMMAND"
T_POS = 0
X_POS = 1
Y_POS = 2
Z_POS = 3

COLS = "t(s)"
TIME = "TIME"
OUT_FILE = "CPS_data.txt"
TESTS = "/test_files"
IMAGES ="Images/"
README = "README.md"
#files = ["1_2018-4-17-13-55-50.txt", "1_2018-4-17-13-58-2.txt","1_2018-4-17-14-32-44.txt","2_2018-4-17-14-1-1.txt","5_2018-4-17-14-3-18.txt", "10_2018-4-17-14-24-58.txt"]
distTimeErrors = []
files=[]
for file in os.listdir(os.getcwd()):
    if not file.endswith(".py") and not file.endswith(".md") and not file.startswith("CPS") and not file.startswith("Images") and not file.startswith("PATH"):
        files.append(file)

x_i = 0
y_i = 0
z_i = 0
x_f = 0
y_f = 0
z_f = 0

m = 0
c = 0

TIME_INDEX = 0
POINT_INDEX = 1
DIST_INDEX = 2
ERROR_INDEX = 3

for file in files:
    newList = []
    isFinalCoord = False
    isInitialCoord = True
    for line in open(file):
        if isFinalCoord:
            isFinalCoord = False
            x_f = float(line.split()[X_POS])
            y_f = float(line.split()[Y_POS])
            z_f = line.split()[3]

        else:
            if isInitialCoord:
                newList = []
                isInitialCoord = False
                isFinalCoord=True
                t = line.split()[T_POS]
                x_i = float(line.split()[X_POS])
                y_i = float(line.split()[Y_POS])
                #z_i = line.split()[3]
                m, c = getLine(x_i, y_i, x_f, y_f)

            else:
                if line.split() is None or len(line.strip()) == 0:
                    distTimeErrors.append(newList)
                    isInitialCoord = True
                    newList = []

                else:
                    t = float(line.split()[T_POS])
                    x = float(line.split()[X_POS])
                    y = float(line.split()[Y_POS])
                    dist = pointDistance(x, y, x_f,y_f)
                    errorDist = lineDistance(m, c, x, y)
                    newList.append([t, (x, y), dist,errorDist])

distTimeErrors.append(newList)
dataFile = open(OUT_FILE, "w")
i = 1

for trial in distTimeErrors:
    dataFile.write("\n"+"Trial: "+str(i)+"\n")
    dataFile.write("time  point  distance from goal    error"+"\n")
    for line in trial:
        dataFile.write(str(line))
        dataFile.write("\n")
    i+=1

count = 0
timeCount=0
for trial in distTimeErrors:
   # print("\n\ntrial "+str(count+1)+"\n\n")
    count+=1
    x = []
    t = []
    for time in trial:
        #print(time)
        x.append(time[ERROR_INDEX])
        #t.append(time[TIME_INDEX])
        t.append(timeCount)
        timeCount+=0.05
    pyplot.clf()
    pyplot.plot(t,x)
    pyplot.xlabel('Time(s)')
    pyplot.ylabel('Distance from straight line')
    pyplot.title('Trial' + str(count)+' Path Error')
    pyplot.grid(True)
    pyplot.savefig(IMAGES+"trial"+str(count)+"error.png")
    pyplot.close()

#plotting
errors = []
maxError = -1
for i in distTimeErrors:
    for j in i:
        errors.append(j[ERROR_INDEX])
        if j[ERROR_INDEX]>maxError:
            maxError = j[ERROR_INDEX]
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
pyplot.savefig(IMAGES+"histogram_all.png")
pyplot.close()


#calc if leave error radius
errorRadii=dict()
errorTransition = dict()
trialsWRadius = dict()
for i in range(0, 15):
    errorRadii[i]=0
    errorRadii[i+.25]=0
    errorRadii[i+.5]=0
    errorRadii[i+.75]=0
for radius in errorRadii:
    for trial in distTimeErrors:
        enters = False
        leavesAfterEnters = False
        initial = True
        for point in trial:
            if initial and point[DIST_INDEX]<=radius:
                # if point[ERROR_INDEX]:
                #     currError = errorTransition[]
                continue
            else:
                if radius in trialsWRadius:
                    trialsWRadius[radius]+=1
                else:
                    trialsWRadius[radius]=1
            initial = False
            if point[DIST_INDEX]<radius:
                enters = True
            if point[DIST_INDEX]>radius and enters:
                leavesAfterEnters=True

        if leavesAfterEnters:
            errorRadii[radius]+=1

pyplot.plot(errorRadii.keys(), errorRadii.values())
pyplot.xlabel("Radius of Safe Zone (m)")
pyplot.ylabel("Trials that left the radius")
pyplot.title("Safe Zone Analysis (counts)")
pyplot.grid(True)
pyplot.savefig(IMAGES+"SafeZoneAnalysis.png")
pyplot.close()

for radius in trialsWRadius:
    if errorRadii[radius]!=0:
        trialsWRadius[radius] = (1.0*trialsWRadius[radius]) / (1.0*errorRadii[radius])
    else:
        trialsWRadius[radius] = 0

pyplot.plot(trialsWRadius.keys(), trialsWRadius.values())
pyplot.xlabel("Radius of Safe Zone (m)")
pyplot.ylabel("Percent of trials that left the radius")
pyplot.title("Safe Zone Analysis (percentage)")
pyplot.grid(True)
pyplot.savefig(IMAGES+"SafeZoneAnalysisPercents.png")
pyplot.close()

#for each distance
#find time of getting to .00 accuracy of position
    #separate into stayed within radius vs left radius
    #Also separate into wavered from path vs didn't?
dataFile = open("PATH ERROR", "w")
pathErrorAnalysis = dict()
for radius in errorRadii:
    leftRadius = dict()
    stayedInRadius = dict()
    for trial in distTimeErrors:
        initial = True
        initialTime = None
        pathDist = None
        enters = False
        leavesAfterEnters = False
        for point in trial:
            if initial:
                initial = False
                initialTime = point[TIME_INDEX]
                pathDist = point[DIST_INDEX]
            else:
                if point[DIST_INDEX]<radius:
                    enters = True
                if point[DIST_INDEX]>radius and enters:
                    leavesAfterEnters=True
                if point[DIST_INDEX]==0:
                    time = point[TIME_INDEX] - initialTime
                    if leavesAfterEnters:
                        if not math.floor(pathDist) in leftRadius:
                            leftRadius[math.floor(pathDist)] = list()
                        leftRadius[math.floor(pathDist)].append(time)
                    else:
                        if not math.floor(pathDist) in stayedInRadius:
                            stayedInRadius[math.floor(pathDist)] = list()
                        stayedInRadius[math.floor(pathDist)].append(time)


    allDistances = set()
    for i in leftRadius.keys():
        allDistances.add(i)
    for i in stayedInRadius.keys():
        allDistances.add(i)
    for pathDist in allDistances:
        numLeft = 0
        numStayed = 0
        percentStayed = 0
        leftTimes = None
        rightTimes = None
        if pathDist in leftRadius:
            numLeft = len(leftRadius[pathDist])
            leftTimes = leftRadius[pathDist]
        if pathDist in stayedInRadius:
            numStayed = len(stayedInRadius[pathDist])
            percentStayed = numStayed*1.0/(1.0*(numLeft+numStayed))
            stayedTimes = stayedInRadius[pathDist]
        minLeft = 0
        maxLeft = 0
        meanLeft = 0
        medianLeft = 0
        if numLeft>0:
            minLeft = min(leftTimes)
            maxLeft = max(leftTimes)
            meanLeft = sum(leftTimes)/numLeft
            medianLeft = leftTimes[math.ceil(numLeft/2)-1]
        minStayed = 0
        maxStayed = 0
        meanStayed = 0
        medianStayed = 0

        if numStayed > 0:
            minStayed = min(stayedTimes)
            maxStayed = max(stayedTimes)
            meanStayed = sum(stayedTimes) / numStayed
            medianStayed = stayedTimes[math.ceil(numStayed / 2) - 1]

        dataFile.write("\n" + "ERROR RADIUS: " + str(radius) + " PATH DISTANCE: "+str(pathDist)+"\n")
        dataFile.write("num Left: "+str(numLeft)+" num stayed: "+str(numStayed) + " percent stayed: "+str(percentStayed)+"\n")
        dataFile.write("min left: "+str(minLeft)+" max left: "+str(maxLeft) + "median left: "+str(medianLeft)+" mean left: "+str(meanLeft)+"\n")
        dataFile.write("min stayed: " + str(minStayed) + " max stayed: " + str(maxStayed) + "median stayed: " + str(
            medianStayed) + " mean stayed: " + str(meanStayed) + "\n")

dataFile.close()
dataFile = open("PATH_ERROR_TWO", "w")
pathErrorAnalysis = dict()
for radius in errorRadii:
    leftRadius = dict()
    stayedInRadius = dict()
    for trial in distTimeErrors:
        initial = True
        initialTime = None
        pathDist = None
        enters = False
        leavesAfterEnters = False
        for point in trial:
            if initial:
                initial = False
                initialTime = point[TIME_INDEX]
                pathDist = point[DIST_INDEX]
            else:
                if point[DIST_INDEX]<radius:
                    enters = True
                if point[DIST_INDEX]>radius and enters:
                    leavesAfterEnters=True
            time = point[TIME_INDEX] - initialTime
        if leavesAfterEnters:
            if not math.floor(pathDist) in leftRadius:
                leftRadius[math.floor(pathDist)] = list()
            leftRadius[math.floor(pathDist)].append(time)
        else:
            if not math.floor(pathDist) in stayedInRadius:
                stayedInRadius[math.floor(pathDist)] = list()
            stayedInRadius[math.floor(pathDist)].append(time)

    allDistances = set()
    for i in leftRadius.keys():
        allDistances.add(i)
    for i in stayedInRadius.keys():
        allDistances.add(i)
        for pathDist in allDistances:
            numLeft = 0
            numStayed = 0
            percentStayed = 0
            leftTimes = None
            rightTimes = None
            if pathDist in leftRadius:
                numLeft = len(leftRadius[pathDist])
                leftTimes = leftRadius[pathDist]
            if pathDist in stayedInRadius:
                numStayed = len(stayedInRadius[pathDist])
                percentStayed = numStayed*1.0/(1.0*(numLeft+numStayed))
                stayedTimes = stayedInRadius[pathDist]
            minLeft = 0
            maxLeft = 0
            meanLeft = 0
            medianLeft = 0
            if numLeft>0:
                minLeft = min(leftTimes)
                maxLeft = max(leftTimes)
                meanLeft = sum(leftTimes)/numLeft
                medianLeft = leftTimes[math.ceil(numLeft/2)-1]
            minStayed = 0
            maxStayed = 0
            meanStayed = 0
            medianStayed = 0

            if numStayed > 0:
                minStayed = min(stayedTimes)
                maxStayed = max(stayedTimes)
                meanStayed = sum(stayedTimes) / numStayed
                medianStayed = stayedTimes[math.ceil(numStayed / 2) - 1]

            dataFile.write("\n" + "ERROR RADIUS: " + str(radius) + " PATH DISTANCE: "+str(pathDist)+"\n")
            dataFile.write("num Left: "+str(numLeft)+" num stayed: "+str(numStayed) + " percent stayed: "+str(percentStayed)+"\n")
            dataFile.write("min left: "+str(minLeft)+" max left: "+str(maxLeft) + "median left: "+str(medianLeft)+" mean left: "+str(meanLeft)+"\n")
            dataFile.write("min stayed: " + str(minStayed) + " max stayed: " + str(maxStayed) + "median stayed: " + str(
                medianStayed) + " mean stayed: " + str(meanStayed) + "\n")


