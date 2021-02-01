import math
from math import pi


xp, yp = 3, 2
r = 1
# theta = 35* 0.0174533

theta = 45 + 0.1
ycl = yp + r*math.cos(math.radians(theta))
ycr = yp - r*math.cos(math.radians(theta))

xcl = xp + math.tan(math.radians(theta))*(yp-ycl)
xcr = xp + math.tan(math.radians(theta))*(yp-ycr)

print("11",xcl,ycl)
print("21",xcr,ycr)

beta = 45

# xcl,ycl=4,2
# xcr,ycr=2,2

#rightChild coordinates:
ynr = ycr + r*math.sin(math.radians(theta+beta))
xnr = xcr - (ycr-ynr)/math.tan(math.radians(theta+beta))
print("right",xnr,ynr)

#left child coordinates:
ynl = ycl + r*math.sin(math.radians(theta-beta))
xnl = xcl - (ycl-ynl)/math.tan(math.radians(theta-beta))
print("left",xnl,ynl)

xcs,ycs = 3,2
yns = ycs + r*math.sin(math.radians(theta))
xns = xcs - (ycs-yns)/math.tan(math.radians(theta))
print('straight',xns,yns)

xon = xo * cos(math.radians(0.1)) - yo * sin(math.radians(0.1))
yon = yo * cos(math.radians(0.1)) + xo * sin(math.radians(0.1))
theta = [45.1, 135.1, 225.1, 315.1]
graph.addVertex((xon, yon))
# theta = [90]
for k in range(len(theta)):
    r = 5
    x1 = xo + r * math.cos(math.radians(theta[k]))
    y1 = yo + r * math.sin(math.radians(theta[k]))
    x1n = x1 * cos(math.radians(0.1)) - y1 * sin(math.radians(0.1))
    y1n = y1 * cos(math.radians(0.1)) + x1 * sin(math.radians(0.1))