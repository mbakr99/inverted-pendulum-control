#! /usr/bin/python3


from numpy import random as random
import numpy as np
from deap import base
from deap import creator
from deap import tools
import rospy 
from std_msgs.msg import Float64
from inverted_pendulum_pkg.srv import CandidateSolution, CandidateSolutionRequest, CandidateSolutionResponse
from inverted_pendulum_pkg.msg import ControllerGains 
import pdb


# helper function
def sum_custom(vals):
    sum = 0
    for x in vals:
        if isinstance(x,tuple):
            sum += x[0]
        else:
            sum += x
    return sum

def mean_vec(vals):
    num_elements = len(vals)
    return sum_custom(vals) / num_elements # E[x]
    
def std_vec(vals):
    num_elements = len(vals)
    mean = sum_custom(vals) / num_elements

    sum_squared_diff = 0
    for x in vals:
        if isinstance(x,tuple):
            sum_squared_diff += (x[0] - mean) ** 2
        else:
            sum_squared_diff += (x - mean) ** 2

    return (sum_squared_diff / num_elements) ** 0.5


# class to handle service call failure 
class ServiceCallFailure(Exception):
    pass

# create the objective class
creator.create("TrackingError",base.Fitness,weights=(-1.0,)) #FIXME:

# create the individual (controller) class
creator.create("Controller",list,fitness=creator.TrackingError)
NUM_CONT_PARAMS=6 # 3 gains for the cart controller and 3 for the pendulum controller 

toolbox=base.Toolbox()
toolbox.register("gains",np.random.uniform,low=0,high=20)
toolbox.register("controller",tools.initRepeat,creator.Controller,toolbox.gains,n=NUM_CONT_PARAMS)
# create a population of controllers
toolbox.register("population",tools.initRepeat,list,toolbox.controller)

# in here, the compute objective client has to call the server with the controller gains
def evaluate_controller(controller):
    srv_msg=CandidateSolutionRequest()
    srv_msg.cart_controller_gains= [0,0,0] # controller[:3] FIXME: 
    srv_msg.pendulum_controller_gains=controller[3:]
    rospy.loginfo("Sending {} controller gains...".format(controller[:]))

    # to handle service failure
    max_retries = 3
    retry_count = 0

    while retry_count < max_retries:
        try:

            # call the service  
            resp=obj_client.call(srv_msg)

            if resp:
                return resp.loss_pend # resp.loss_cart FIXME:
            else:
                raise ServiceCallFailure("Service was called but return false success flag")
        
        # handle exceptions 
        except (rospy.ServiceException, ServiceCallFailure) as e:
            retry_count += 1
            rospy.logwarn(rospy.logwarn(f"Service call failed (attempt {retry_count}/{max_retries}): {e}"))
            
    rospy.logwarn(f"Retunring large valaues for the objective since the call to the compute_objective failed {max_retries} times.")
    return 1e6 # 1e6 FIXME:


#define the opertors 
toolbox.register("evaluate", evaluate_controller)
toolbox.register("select", tools.selTournament, tournsize=4)
toolbox.register("mate", tools.cxBlend, alpha=0.2) #I will start with a simple cross-over function (might try more complex ones if needed)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2) 



if __name__ == "__main__":

    rospy.init_node("optimizer_node")
    rospy.wait_for_service("inverted_pendulum/compute_objective")
    rospy.loginfo("compute objective service is running")
    try:
        obj_client=rospy.ServiceProxy("inverted_pendulum/compute_objective",CandidateSolution)
    except:
        rospy.logwarn("The clinet was not able to connect to the server")

    pend_controller_gains_publisher = rospy.Publisher("/inverted_pendulum/pend_controller_gains", ControllerGains, queue_size=10)
    cart_controller_gains_publisher = rospy.Publisher("/inverted_pendulum/cart_controller_gains", ControllerGains, queue_size=10) 

    pend_error_publisher = rospy.Publisher("/inverted_pendulum/pend_error", Float64, queue_size=10)
    cart_error_publisher = rospy.Publisher("/inverted_pendulum/cart_error", Float64, queue_size=10)

        




    NGEN=30
    CXPB=0.6
    MUTPB=0.5

    pop=toolbox.population(100)
    pop_fitness=list(map(toolbox.evaluate,pop))
    # pdb.set_trace() # FIXME: 
    for ind,fit in zip(pop,pop_fitness):
        ind.fitness.values=[fit]

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
        new_controllers = [ind for ind in offspring if not ind.fitness.valid]
        new_offspring_fitness = list(map(toolbox.evaluate, new_controllers))
        for new_controller, fitness in zip(new_controllers, new_offspring_fitness):
            new_controller.fitness.values = [fitness]

        # Replace the old population with the new offspring
        pop[:] = offspring

        new_population_fitness_value = [ind.fitness.values for ind in pop] # FIXME: for some reason the previous code was indexing the first element of the fitness value ind.fitness.values[0]
        pop_size = len(pop)
        min_cost_pend = min(new_population_fitness_value)
        max_cost_pend = max(new_population_fitness_value)
        mean_cost_pend = mean_vec(new_population_fitness_value)
        std_cost_pend = std_vec(new_population_fitness_value)


        print(f"  Min (pend, cart): {min_cost_pend}")
        print(f"  Max (pend, cart): {max_cost_pend}")
        print(f"  Avg (pend, cart): {mean_cost_pend}")
        print(f"  Std (pend, cart): {std_cost_pend}")

        # examine the fittest controller 
        # best_controller_idx = np.argmin([pend_loss + cart_loss for pend_loss ,cart_loss in new_population_fitness_value])
        # best_controller = pop[best_controller_idx]

        # # get the loss associated with best controller 
        # loss = new_population_fitness_value[best_controller_idx]
        # rospy.loginfo(f"The best loss is {loss}")
        
        # # publish the controller info 
        # controller_info_msg = ControllerGains()
        # controller_info_msg.controller_name = "cart"
        # controller_info_msg.gains = best_controller[:3]
        # cart_controller_gains_publisher.publish(controller_info_msg)

        # controller_info_msg.controller_name = "pendulum"
        # controller_info_msg.gains = best_controller[3:]
        # pend_controller_gains_publisher.publish(controller_info_msg)
        

        

        # pend_error_publisher.publish()

       # choosing the best individual
    best_ind = tools.selBest(pop, 1)[0]
    print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))


    


