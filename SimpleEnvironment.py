import numpy
import matplotlib.pyplot as pl
import time

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        config[0] = numpy.random.uniform(low=lower_limits[0], high=upper_limits[0]);
        config[1] = numpy.random.uniform(low=lower_limits[1], high=upper_limits[1]);
        
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return numpy.linalg.norm(numpy.array(start_config) - numpy.array(end_config));

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        origTransform = self.robot.GetTransform()

        steps = 10;

        xSteps = numpy.linspace(start_config[0], end_config[0], (steps + 1));
        ySteps = numpy.linspace(start_config[1], end_config[1], (steps + 1));

        for i in range(steps + 1):

            transform = self.robot.GetTransform()
            transform[0, 3] = xSteps[i]
            transform[1, 3] = ySteps[i]
            self.robot.SetTransform(transform);
            

            # for body in self.robot.GetEnv().GetBodies():
            #     if (body.GetName() != self.robot.GetName() and
            #             self.robot.GetEnv().CheckCollision(body, self.robot)):
                    #self.robot.SetTransform(origTransform);
            if (self.robot.GetEnv().CheckCollision(self.robot) or
                self.robot.CheckSelfCollision()):
                if (i == 0):
                    return None;
                else:
                    return [xSteps[i-1], ySteps[i-1]]

        #self.robot.SetTransform(origTransform)

        return end_config

    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        initTime = time.time()
        while(time.time()-initTime<timeout):
	    ind = 1
	    while(ind < len(path)-1):
                start_config = path[ind-1]
                final_config = path[ind+1]
                config = self.Extend(start_config,final_config)
                
		if config != None:
                    same_bool = 1
                    for i in range(len(config)):
                        if config[i]!=final_config[i]:
                            same_bool = 0
                    if same_bool ==1:
                        del path[ind]
	        ind = ind + 1
		
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

