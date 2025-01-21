#! /usr/bin/python3


from numpy import random as random
import numpy as np
from deap import base
from deap import creator
from deap import tools
import rospy 
from inverted_pendulum_pkg.srv import CandidateSolution, CandidateSolutionRequest, CandidateSolutionResponse


#create the objective class
creator.create("TrackingError",base.Fitness,weights=(-1.0,-1.0))

#create the individual (controller) class
creator.create("Controller",list,fitness=creator.TrackingError)
NUM_CONT_PARAMS=6 # 3 gains for the cart controller and 3 for the pendulum controller 

toolbox=base.Toolbox()
toolbox.register("gains",np.random.uniform,low=0,high=20)
toolbox.register("controller",tools.initRepeat,creator.Controller,toolbox.gains,n=NUM_CONT_PARAMS)
#create a population of controllers
toolbox.register("population",tools.initRepeat,list,toolbox.controller)

#In here, the compute objective client has to call the server with the controller gains
def evaluate_controller(controller):
    srv_msg=CandidateSolutionRequest()
    srv_msg.cart_controller_gains=controller[:3] 
    srv_msg.pendulum_controller_gains=controller[3:]
    rospy.loginfo("Sending {} controller gains...".format(controller[:]))
    resp=obj_client.call(srv_msg)
    rospy.loginfo("The pendulum loss was: {}, and {}".format(resp.loss_pend,resp.loss_cart))
    return resp.loss_pend,resp.loss_cart

#define the opertors 
toolbox.register("evaluate",evaluate_controller)
toolbox.register("select",tools.selTournament,tournsize=4)
toolbox.register("mate",tools.cxTwoPoint) #I will start with a simple cross-over function (might try more complex ones if needed)
toolbox.register("mutate",tools.mutGaussian,mu=0,sigma=1,indpb=0.2) 



if __name__ == "__main__":

    rospy.init_node("optimizer_node")
    rospy.wait_for_service("inverted_pendulum/compute_objective")
    rospy.loginfo("compute objective service is running")
    try:
        obj_client=rospy.ServiceProxy("inverted_pendulum/compute_objective",CandidateSolution)
    except:
        rospy.logwarn("The clinet was not able to connect to the server")

        




    NGEN=60
    CXPB=0.6
    MUTPB=0.3

    pop=toolbox.population(40)
    pop_fitness=list(map(toolbox.evaluate,pop))

    for ind,fit in zip(pop,pop_fitness):
        ind.fitness.values=fit

    for g in range(NGEN):
        rospy.loginfo("Generation (%d)",g)
        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CXPB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        # Apply mutation on the offspring
        for mutant in offspring:
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        unevaluated_controllers = [ind for ind in offspring if not ind.fitness.valid]
        their_fitness = list(map(toolbox.evaluate, unevaluated_controllers))
        for cont, fitness in zip(unevaluated_controllers, their_fitness):
            cont.fitness.values = fitness

        # Replace the old population with the new offspring
        pop[:] = offspring

        fit_report = [ind.fitness.values[0] for ind in pop]
        length = len(pop)
        mean = sum(fit_report) / length
        sum2 = sum(x * x for x in fit_report) #E[x^2] 
        std = abs(sum2 / length - mean**2)**0.5 # E[x^2] - (E[x])^2

        print("  Min %s" % min(fit_report))
        print("  Max %s" % max(fit_report))
        print("  Avg %s" % mean)
        print("  Std %s" % std)





    


