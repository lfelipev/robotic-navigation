import cv2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

class Map():
    def __init__(self, q_goal, zeta=0.05, d_star=2, Q_star=50, n=5):
        self.imgShape = self.loadImg()
        self.q_goal = q_goal
        self.zeta = zeta
        self.d_star = d_star
        self.Q_star = Q_star
        self.n = n
        self.U_att = np.zeros((self.imgShape[0], self.imgShape[1]))
    
    def attractivePotential(self):
        for x in range(0, self.imgShape[0]):
            for y in range(0, self.imgShape[1]):
                d = np.sqrt((self.q_goal[0] - x)**2 + (self.q_goal[1] - y)**2)

                if(d <= self.d_star):
                    self.U_att[x, y] = 0.5 * self.zeta * d**2
                else:
                    self.U_att[x, y] = (self.d_star * self.zeta * d) - (0.5 * self.zeta * (self.d_star)**2)

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        w = np.arange(1, self.imgShape[0], 0.1)
        h = np.arange(1, self.imgShape[1], 0.1)
        X, Y = np.meshgrid(w, h)
        print(np.shape(self.U_att))
        #Z = np.reshape(self.U_att, np.shape(X))
        #surf = ax.plot_surface(X, Y, Z)

        #surf = ax.plot_surface(X, Y, self.U_att, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        #fig.colorbar(surf, shrink=0.5, aspect=5)
        plt.show()
        

    def loadImg(self):
        map = cv2.imread('mapa.png', 0)
        return [map.shape[0], map.shape[1]]

def showImage(img):
    cv2.imshow('mapa', img)

    while True:
        # press 'esc' to exit
        k = cv2.waitKey(0) & 0xFF
        if k == 27: break
    cv2.destroyAllWindows()

def loadImg():
    map = cv2.imread('mapa.png', 0)
    x = map.shape[0]
    print(x)
    #showImage(map)

def plotField():
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    X = np.arange(-5, 5, 0.25)
    Y = np.arange(-5, 5, 0.25)
    X, Y = np.meshgrid(X, Y)
    R = np.sqrt(X**2 + Y**2)
    Z = np.sin(R)

    surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                            linewidth=0, antialiased=False)
    fig.colorbar(surf, shrink=0.5, aspect=5)

    plt.show()


def main():
    q_goal = [65, 41]
    map = Map(q_goal)
    map.attractivePotential()

if __name__ == '__main__':
    main()


