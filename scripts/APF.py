import numpy as np
from matplotlib import pyplot as plt 

class APFHelper:
    def __init__(self, coef_att, coef_rep, threshold_dist):
        self.coef_att = coef_att
        self.coef_rep = coef_rep
        self.threshold_dist = threshold_dist
    
    def GetAttraction(self, coordQuad, coordDest):
        '''Obtain the attraction of the current location
        
        Arguments:
        coordQuad -- the coordinate of the quadroptor, in numpy vector
        coordDest -- the coordinate of the destination, in numpy vector
        
        Return:
        the vector of attraction force
        '''
        return (coordDest - coordQuad) * self.coef_att;
    
    def GetRepulsion(self, coordQuad, obstList):
        '''Obtain the replusion of the current location
        
        Arguments:
        coordQuad -- the coordinate of the quadroptor, in numpy vector
        obstList -- the list of coordinates of obstacles, in matrix n*2
        
        Return:
        the vector of repulsion force
        '''
        
        rep = np.array([0.,0.])
        for obstacle in obstList:
            disVec = coordQuad - obstacle
            #print("obstacle ", obstacle, " vector ", disVec)
            disVecNorm = np.linalg.norm(disVec)
            
            if disVecNorm > self.threshold_dist:
                continue
            
            disVecUniform = disVec * (1. / disVecNorm)
            delta_rep = disVecUniform * self.coef_rep * (1./disVecNorm - 1./self.threshold_dist) * (1./(disVecNorm ** 2.))
            rep += delta_rep
            
        return rep
        
class APFPlanner:
    
    def __init__(self, helper, max_iteration = 10000, march = 0.01, tolerance = 0.1):
        self.helper = helper;
        self.max_iteration = max_iteration
        self.march = march
        self.tolerance = tolerance
        
    def Planning(self, coordStart, coordDest, obstList):
    
        iteration = 0
        successful = False
        coordCurrent = coordStart
        path = []
        
        print("Starting from ", coordCurrent)
        
        while(iteration < self.max_iteration):
            iteration += 1
            
            if(np.linalg.norm(coordCurrent - coordDest) < self.tolerance):
                successful = True
                break
            
            #print("Attraction ", helper.GetAttraction(coordCurrent, coordDest))
            #print("Repulsion ", helper.GetRepulsion(coordCurrent, obstList))
            force = helper.GetAttraction(coordCurrent, coordDest) + helper.GetRepulsion(coordCurrent, obstList);
            coordCurrent += force * (self.march / np.linalg.norm(force))
            #print("Reaching ", coordCurrent)
            path.append((coordCurrent[0], coordCurrent[1]))
            
        return (successful, iteration, path)
        
if __name__ == "__main__":

    helper = APFHelper(1, 10.0, 3)
    planner = APFPlanner(helper, 5000)
    
    start = np.array([0.,0.])
    goal = np.array([15.,15.])
    
    obs = np.array([[4.,3.], [7.,8.], [9., 10.], [5., 7.], [13., 8.]])
    
    result = planner.Planning(start, goal, obs)
    if result :
        print("Successful after", result[1], "ierations")
        
    plt.plot(start[0], start[1])
    plt.plot(goal[0], goal[1])
    for obstacle in obs:
        plt.plot(obstacle[0], obstacle[1], 'xr')
    for path in result[2]:
        #print(path[0], path[1])
        plt.plot(path[0], path[1], '.b')
    plt.show()
        
