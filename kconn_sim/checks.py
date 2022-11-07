
import numpy as np
import math
def check_destination(agent, tasks):

    possible_task_i = np.arange(len(tasks))
    if math.dist( agent.location, agent.task ) < 10:
        if np.where(possible_task_i == agent.task_index)[0][0] == len(tasks)-1:
            agent.task = tasks[0]
            agent.task_index = 0
        else:
            agent.task = tasks[agent.task_index + 1]
            agent.task_index = agent.task_index + 1







          
             
