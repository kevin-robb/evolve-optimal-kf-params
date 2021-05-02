# This file creates and manages the set of agents, and assigns fitness.

from functions import read_results
from ec_agent import Agent
from typing import List, Tuple
from random import choices
from datetime import datetime
import subprocess
import sys

def initialize_agents(roster_size:int, next_id:List[int], rand:bool) -> List:
    roster = []
    for _ in range(roster_size):
        roster.append(Agent(id=next_id[0],rand=rand))
        next_id[0] += 1
    return roster

def next_generation(roster:List, next_id:List[int]) -> List:
    # sort this generation by fitness. (smaller = better)
    roster.sort(key=lambda a: a.fitness) #reverse=True
    # define the weights for each position. for now, simple linear decrease.
    r_weights = [len(roster)-i for i in range(len(roster))]
    # select double what is needed to parent the next generation. (agents can/will be multi-selected)
    parents = choices(roster, weights=r_weights, k=2*len(roster))
    # create the next generation by crossing over pairs of parents.
    next_gen = [parents[i].crossover(parents[i + len(roster)], next_id) for i in range(len(roster))]
    return next_gen

def setup_summary_file(directory:str) -> str:
    filepath = directory + "/summary.csv"
    header = "agent_id,generation_number,fitness,"
    # include the genome labels.
    file2 = open("config/default_genome.csv","r+")
    header += file2.readline()
    file2.close()
    # create the file and write the header to the first line.
    file1 = open(filepath, "a+")
    file1.write(header) #+"\n"
    file1.close()
    return filepath

def setup_dir() -> Tuple:
    # make a unique ID for tracking this run's files.
    dt = datetime.now()
    run_id = dt.strftime("%Y-%m-%d-%H-%M-%S")
    # make a directory to store KF data, gen summaries, and plots.
    directory = "runs/run_" + run_id
    run_bash_cmd("mkdir " + directory)
    # create the summary file for this run and return the filepath.
    return directory, setup_summary_file(directory)

def set_kf_data_loc(directory:str, fname:str):
    filepath = "config/kf_data_destination.csv"
    # make sure directory ends in "/"
    if directory[-1] != "/": 
        directory += "/"
    file1 = open(filepath, "w")
    row = directory + fname
    file1.write(row)
    file1.close
    return row

def run_bash_cmd(command:str):
    #print("Running cmd: " + command)
    # run something on the command line.
    process = subprocess.Popen(command.split())
    #process = subprocess.Popen(command.split(), stdout=subprocess.PIPE, shell=True)
    output, error = process.communicate()
    #system(command)
    
def main():
    # set parameters from cmd line args.
    if len(sys.argv) > 1:
        # number of agents in a generation.
        gen_size = int(sys.argv[1])
        # number of generations to run.
        num_gens = int(sys.argv[2])
    else:
        # ask to input them (probably I just forgot).
        gen_size = int(input("Enter number of agents per gen: "))
        num_gens = int(input("Enter number of generations to run: "))
        rand = None
        while rand is None:
            temp = input("Full random instead of basing on default genome? (true/false): ")
            if temp.lower() == "true":
                rand = True
            elif temp.lower() == "false":
                rand = False
            else:
                print("Please enter 'true' or 'false'.")
        # see if they skipped this, and use default values if so.
        if gen_size is None or num_gens is None:
            gen_size, num_gens = 3, 3
        # verify the chosen option.
        if rand:
            print("Using random initial genes.")
        else:
            print("Using decent starting genome.")

    # source the ROS workspace.
    #run_bash_cmd("source sim_ws/devel/setup.bash")
            
    # create the folder/file we will use for this run's data.
    directory, summary_filepath = setup_dir()
    # ensure each agent gets a different ID. 
    # this is a list with one element so it can be changed inside functions.
    next_id = [1]
    # create the first generation of agents, with default values.
    roster = initialize_agents(gen_size, next_id, rand)

    # run the sim for each agent to obtain fitness for each.
    while roster[0].gen_num <= num_gens:
        # TODO look into using the same seed for agents in the same generation.
        for agent in roster:
            # set the KF to use this agent's genome params.
            agent.set_genome()
            # make sure the KF outputs data to the right place.
            agent_filename = "kf_output_" + str(agent.id)
            fpath = set_kf_data_loc(directory, agent_filename)
            # call the bash script to run the sim.
            run_bash_cmd("bash run_once.sh nondefault")
            # assign the agent a fitness based on the results.
            agent.results = read_results.read_file()
            agent.calc_fitness(fpath, directory)
            # write this agent & its performance to the files.
            agent.write_to_file(summary_filepath)
            read_results.write_file(agent.results)
            # generate the plot for this agent's KF data.
            run_bash_cmd("Rscript --vanilla functions/plot_one_run.R " + agent_filename + " " + directory)

        # form the next generation.
        roster = next_generation(roster, next_id)

    # after repeating for the desired number of generations, run our plotting script.
    run_bash_cmd("Rscript --vanilla functions/plot_summary.R " + directory)

    # TODO make individual plots for comparison with best/worst/avg agents in first/mid/final gen.

if __name__ == "__main__":
    main()
