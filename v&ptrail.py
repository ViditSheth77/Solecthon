import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from operator import itemgetter

L1 = []
L = []
xs = []
ys = []
zs = []
vs = []
mx = []
my = []
m = []
x = 18
left = []
right = []
mybox = [[3, 1.5], [5.5, 1.5], [3, 3], [5.5, 3], [2.5, 4.5], [5, 4.5], [2, 6], [4.5, 6], [1.5, 7.5], [4, 7.5], [1, 9],
         [3.5, 9], [1, 10.5], [3.5, 10.5], [2, 12], [4.5, 12], [2, 13.5], [4.5, 13.5]]
while True:
    """data = file.readlines(x)
    if not data :
        break
    for s in range(len(data)):
        if '\n' in data[s] :
            L.append(data[s][:-1])
        else :
            L.append(data[s])"""
    # print(L)
    """for i in mybox :
        #L1.append(("" + i).split(" "))
        mybox.sort(key=lambda k: [k[1], k[0]])"""
    sorted(mybox, key=itemgetter(0, 1))
    D = 0
    A = len(mybox)

    # start writing here computation
    for i in range(len(mybox)):
        for j in range(1, len(mybox)):
            # print("mybox[j][1]=", mybox[j][1])
            D = (((float(mybox[j][0]) - float(mybox[i][0])) * 2) + ((float(mybox[j][1]) - float(mybox[i][1]))**2)) * 0.5
            # print("D = ", D)
            if D == 2.5:
                if float(mybox[i][0]) > float(mybox[j][0]) and mybox[j] not in left and mybox[i] not in right:
                    print("mybox[i]=", mybox[i])
                    left.append(mybox[i])
                    right.append(mybox[j])
                elif float(mybox[j][0]) > float(mybox[i][0]) and mybox[i] not in left and mybox[j] not in right:
                    right.append(mybox[j])
                    left.append(mybox[i])
    print("left", left)
    print("right", right)
    for i in left:
        xs.append(i[0])
        ys.append(i[1])
    print("xs=", xs)
    print("ys=", ys)
    for i in range(len(xs)):
        plt.scatter(float(xs[i]), float(ys[i]), label="stars", color="green", marker="1", s=30)
        plt.xlim(0, 15)
        plt.ylim(0, 15)
    for j in right:
        zs.append(j[0])
        vs.append(j[1])
    print("zs=", zs)
    print("vs=", vs)
    for j in range(len(zs)):
        plt.scatter(float(zs[j]), float(vs[j]), label="stars", color="red", marker="1", s=30)
        plt.xlim(0, 15)
        plt.ylim(0, 15)
    for i in range(0, len(xs)):
        mx.append((xs[i]+zs[i])/2)
        my.append((ys[i]+vs[i])/2)
        """mx[i].append(((float(xs[i]) + float(zs[i])) / 2))
        my[i].append(((float(ys[i]) + float(vs[i])) / 2))"""
        plt.scatter(float((xs[i]+zs[i])/2), float((ys[i]+vs[i])/2), label="stars", color="blue", marker="1", s=30)
        """print("mx=", mx)
        print("my=", my)"""
    print("mx=", mx)
    print("my=", my)
    break 
plt.show()
mybox.clear()
L.clear()
left.clear()
right.clear()
xs.clear()
ys.clear()
zs.clear()
vs.clear()
mx.clear()
my.clear()
# print("clear")