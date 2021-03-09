# This file creates and manages the set of agents, and assigns fitness.

from functions import read_results
from ec_agent import Agent
from typing import Dict, List
from random import choices
from datetime import datetime
import subprocess

def initialize_agents(roster_size:int) -> List:
    roster = []
    for i in range(roster_size):
        roster.append(Agent())
    return roster

def next_generation(roster:List):
    # sort this generation by fitness. (smaller = better)
    roster.sort(key=lambda a: a.fitness) #reverse=True
    # define the weights for each position. for now, simple linear decrease.
    r_weights = [len(roster)-i for i in range(len(roster))]
    # select double what is needed to parent the next generation. (agents can/will be multi-selected)
    parents = choices(roster, weights=r_weights, k=2*len(roster))
    # create the next generation by crossing over pairs of parents.
    next_gen = [parents[i].crossover(parents[i + len(roster)]) for i in range(len(roster))]
    return next_gen

def setup_file() -> str:
    # make a unique ID for tracking this run's files.
    dt = datetime.now()
    run_id = "ec_" + dt.strftime("%Y-%m-%d-%H-%M-%S")
    filepath = "results/" + run_id + ".csv"
    # create the file and write the header to the first line.
    header = ["generation_number","p_11","p_22","p_33","p_44","q_11","q_22","q_33","q_44","r_11","r_22","r_33","r_44","fitness"]
    file1 = open(filepath, "a+")
    file1.write(",".join(header) + "\n")
    return run_id
    
def main():
    # size of each gen, # of gens to run for
    gen_size, gen_max = 5, 3
    run_id = setup_file()
    
    # create the first generation of agents, with default values.
    roster = initialize_agents(gen_size)
    while roster[0].gen_num <= gen_max:
        # run the sim for each agent to obtain fitness for each.
        # TODO look into using the same seed for agents in the same generation.
        for agent in roster:
            # set the KF to use this agent's genome params.
            agent.set_genome()
            # call the bash script to run the sim.
            run_sim_bash = "bash run_sim.sh"
            process = subprocess.Popen(run_sim_bash.split(), stdout=subprocess.PIPE) #cwd='path\to\somewhere'
            output, error = process.communicate()
            # assign the agent a fitness based on the results.
            agent.results = read_results.read_file()
            agent.fitness = agent.results["Score"]
            # write this agent & its performance to the file.
            agent.write_to_file(run_id)
        # form the next generation.
        roster = next_generation(roster)

    # after repeating for the desired number of generations, run our plotting script.
    # TODO make an R plotting script and run it here with bash.
    # plot the evolution of agents' fitness as well as 
    #   the best agent in the final generation using my previous in-depth script.

if __name__ == "__main__":
    main()
