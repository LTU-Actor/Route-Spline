import matplotlib
import matplotlib.pyplot as plt
import math
import functools

colors = ['red', 'teal', 'green', 'purple', 'grey', 'blue', 'yellow']

#returns the distance from p to f(t)
def  distance(p,f):
    def mult(args):
        x,y = args
        return (x-y)**2

    args = list(zip(p,f))

    return math.sqrt(sum(map(mult, args)))

#returns the first derivative the the distance from p to f(t)
def distanceFirstDerivative(p,f,fd):
    def firstDerivative(args):
        p,f,fd = args 
        return fd*(f-p) 

    args = list(zip(p,f,fd))
    return sum(map(firstDerivative, args))/distance(p,f)

#returns the second derivative the the distance from p to f(t)
def distanceSecondDerivative(p,f,fd,fdd):
    def secondDerivativePart1(args):
        p,f,fd,fdd  = args
        return (f - p)*fdd + fd**2
    
    def secondDerivativePart2(args):
        p,f,fd,fdd = args
        return (f-p)*fd

    args = list(zip(p,f,fd,fdd))

    part1 = sum(map(secondDerivativePart1, args))/distance(p,f)
    part2 =  (sum(map(secondDerivativePart2, args))**2)/(distance(p,f)**3)
    return part1+part2

class spline:
    #draws a spline between p1 and p2
    def __init__(self, p0, p1, p2, p3):
        self.xs, self.ys = zip(*[p0,p1,p2,p3])

        self.xy = [self.xs,self.ys]

        # 1/2( ((-p0 + 3p1 -3p2 + p3)u + (2p0 -5p1 + 4p2 - p3))u + (-p0 + p3) )u + p1

        def e1(arr):
            return 0.5*(-arr[0] + 3*arr[1] - 3*arr[2] + arr[3])

        def e2(arr):
            return 0.5*(2*arr[0] - 5*arr[1] + 4*arr[2] - arr[3])

        def e3(arr):
            return 0.5*(-arr[0] + arr[2])

        self.eq1 = list(map(e1, self.xy))
        self.eq2 = list(map(e2, self.xy))
        self.eq3 = list(map(e3, self.xy))

        self.eq_xs, self.eq_ys = zip(*(self.eq1,self.eq2,self.eq3))
        self.eq_xy = [self.eq_xs, self.eq_ys]

    def getPoint(self, t):
        #t is im the interval [0,1)
        def getp(args):
            e1, e2, e3, p ,t = args
            return ((e1*t + e2)*t + e3)*t + p

        args = [[*self.eq_xs, self.xs[1], t],[*self.eq_ys, self.ys[1], t]]
        return list(map(getp, args))

    def getFirstDerivativePoint(self, t):
        def getpd(args):
            e1, e2, e3,t = args
            return ((3*e1*t + 2*e2)*t + e3)

        args = [[*self.eq_xs, t],[*self.eq_ys, t]]
        
        return list(map(getpd, args))

    def getSecondDerivativePoint(self, t):
        def getpdd(args):
            e1, e2, t = args
            return (6*e1*t + 2*e2)
        
        args = [[*self.eq_xs[:-1], t],[*self.eq_ys[:-1], t]]

        return list(map(getpdd, args))

    def drawSpline(self, c):
        res = 0.01
        t = 0
        arr = []

        while (t < 1):
            arr.append(self.getPoint(t))
            t += res

        xs = [p[0] for p in arr]
        ys = [p[1] for p in arr]

        plt.plot(xs, ys, 'ro', color=c)

    def findClosestPoint(self, pos):
    
        t = 0.5
        f = self.getPoint(t)

        if distance(pos, f) == 0:
            return t

        fd = self.getFirstDerivativePoint(t)
        fdd = self.getSecondDerivativePoint(t)

        dd = distanceFirstDerivative(pos,f,fd)
        ddd = distanceSecondDerivative(pos, f, fd, fdd)

        while (abs(dd) > 0.001 and t >= 0 and t < 1):
            t = t - dd/ddd

            f = self.getPoint(t)

            if distance(pos, f) == 0:
                return t

            fd = self.getFirstDerivativePoint(t)
            fdd = self.getSecondDerivativePoint(t)

            dd = distanceFirstDerivative(pos,f,fd)
            ddd = distanceSecondDerivative(pos, f, fd, fdd)

        if t < 0:
            t = 0

        if t > 1:
            t = 1


        return t


def drawLine(point, slope, length, c):
    length = math.floor(length/2)
    points = []
    slope = [-slope[1]/distance(slope,[0,0])/3, slope[0]/distance(slope,[0,0])/3]
    i = 0
    while (i < length):
        points.append([point[0] + (i)*slope[0], point[1] + (i)*slope[1]])
        points.append([point[0] - (i)*slope[0], point[1] - (i)*slope[1]])
        i+=0.01

    points = zip(*points)

    plt.plot(*points, 'ro', color=c)


points = [[0.0,0.0], [2.5,5.0], [7.5,5.0], [10,4], [10.0,0.0],[15.0, 2.0], [17.0, 7.0], [20, 5], [18, 1]]

p = [7,7]
splines = []
distances = []
distance_points = []

for i in range(len(points) - 3):    
    splines.append(spline(*points[i:i+4]))

closest_time = splines[0].findClosestPoint(p) 
closest_distance = distance(p,splines[0].getPoint(closest_time))
spline_num = 0

tot = 10

while (tot != 0):

    closest_time = splines[0].findClosestPoint(p) 
    closest_distance = distance(p,splines[0].getPoint(closest_time))
    spline_num = 0

    for i in range(len(splines)):
        splines[i].drawSpline('black')
        drawLine(splines[i].getPoint(0), splines[i].getFirstDerivativePoint(0), 5, 'blue')
        ct = splines[i].findClosestPoint(p)

        if (ct >= 0 and ct < 1):
            cp = splines[i].getPoint(ct)
            cd = distance(p,cp)

            if cd < closest_distance:
                closest_distance = cd
                closest_time = ct
                spline_num = i

    closest_point = splines[spline_num].getPoint(closest_time)

    plt.plot(*p, 'ro', color='orange')
    plt.plot(*splines[spline_num].getPoint(closest_time), 'ro', color='red')

    slope = [closest_point[0] - p[0], closest_point[1] - p[1]]

    #not wokring

    dirv = splines[spline_num].getFirstDerivativePoint(closest_time)

    #slope = [slope[0] + dirv[0], slope[1] + dirv[0]]
    #print(dirv)

    #working

    mag = distance(slope, [0,0])
    scale = 0.6
    if mag > scale:
        slope = list(map(lambda x: x/mag*scale,slope))
    print(slope)
    dist = distance([0,0], dirv)
    slope = [slope[0] + dirv[0]/dist*scale/1.5, slope[1] + dirv[1]/dist*scale/1.5]
    p = [p[0] + slope[0], p[1] + slope[1]]
    tot -= 1
    plt.show()

