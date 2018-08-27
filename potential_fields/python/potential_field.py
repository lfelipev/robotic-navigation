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
        self.U_rep = self.U_att
        self.map = cv2.imread('mapa.png', 0)

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
        w = np.arange(0, self.imgShape[0], 1)
        h = np.arange(0, self.imgShape[1], 1)
        X, Y = np.meshgrid(w, h)

        surf = ax.plot_surface(X, Y, np.transpose(self.U_att))
        plt.show()
    
    def callMesh(self):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        w = np.arange(0, self.imgShape[0], 1)
        h = np.arange(0, self.imgShape[1], 1)
        X, Y = np.meshgrid(w, h)

        surf = ax.plot_surface(X, Y, np.transpose(self.U_att))
        plt.show()

    def safeDivide(self, x, y):
        if y == 0:
            return 0
        return x / y
    
    def repulsivePotential(self):
        height = self.imgShape[1]
        width = self.imgShape[0]
        for y in range(0, height):
            for x in range(0, width):
                d = self.Q_star

                range_x = [x - (self.Q_star/2), x + (self.Q_star/2)]
                range_y = [y - (self.Q_star/2), y + (self.Q_star/2)]

                if(range_y[0] <= 0):
                    range_y[0] = 1
                if(range_x[0] <= 0):
                    range_x[0] = 1
                if(range_y[1] > height):
                    range_y[1] = height
                if(range_x[1] > width):
                    range_x[1] = width

                for y0 in range(int(range_y[0]), int(range_y[1])):
                    for x0 in range(int(range_x[0]), int(range_x[1])):
                        #print(np.shape(self.map))
                        if(self.map[x0, y0] < 255):
                            d_i = np.sqrt((x-x0)**2 + (y-y0)**2)
                        
                            if(d_i < d):
                                d = d_i
                                
                
                
                self.U_rep[y, x] = 0.5 * self.n * (self.safeDivide(1, d) - (1/self.Q_star))**2
               
                sat = 50

                if(self.U_rep[y, x] > sat):
                    self.U_rep[y, x] = sat

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        w = np.arange(0, self.imgShape[0], 1)
        h = np.arange(0, self.imgShape[1], 1)
        X, Y = np.meshgrid(w, h)

        surf = ax.plot_surface(X, Y, np.transpose(self.U_rep))
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
    #map.attractivePotential()
    map.repulsivePotential()

if __name__ == '__main__':
    main()


