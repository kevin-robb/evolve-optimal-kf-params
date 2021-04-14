# this file defines our evolutionary computation agents
from typing import List, Tuple
from random import random, uniform, randint, choice
from os import stat
from numpy.random import normal

class Agent:
    # instantiate an agent, either with the default genome or a given one.
    def __init__(self, id:int, genome:List[float] = None, gen_num:int = 1):
        self.id = id
        self.fitness = -1
        self.results = {}
        self.gen_num = gen_num
        if genome is not None:
            self.genome = genome
        else:
            self.init_genome()
            self.randomize_genome()
    
    # grab the default genome values from the config file.
    def init_genome(self):
        filepath = "config/default_genome.csv"
        file3 = open(filepath, "r")
        self.genome = [float(g) for g in file3.readlines()[1].split(",")]
        file3.close()
    
    # randomize the genome for the first generation.
    def randomize_genome(self):
        # use a gaussian w/ mean 0 and std dev 0.1 to change each gene.
        self.genome = [max(abs(g + normal(loc=0, scale=0.01)), 0.005) for g in self.genome]

    # mutate the genome of this agent.
    def mutate(self):
        # choose a gene to mutate.
        gene = randint(0,len(self.genome)-1)
        # choose the amount to mutate (gaussian, mean 0, std dev 0.1) and do it.
        # make sure it is positive, and that the value doesn't drop too close to 0.
        self.genome[gene] = max(abs(self.genome[gene] + normal(loc=0, scale=0.01)), 0.005)

    # cross the genes of this agent with another to produce a child.
    def crossover(self, other_parent:"Agent", next_id:List[int]) -> "Agent":
        # decide which genes will be taken from which parent.
        new_genome = [self.genome[gene] if randint(0,1) < 1 else other_parent.genome[gene] for gene in range(len(self.genome))]
        child = Agent(id=next_id[0], genome=new_genome, gen_num=self.gen_num+1)
        next_id[0] += 1
        child.mutate()
        return child

    # # perform crossover, but do not separate groups of genes belonging to the same matrix.
    # def group_crossover(self, other_parent:"Agent", next_id:List[int]) -> "Agent":
    #     # arbitrarily take gene groups from one parent each.
    #     selections = [choice([True, False]) for i in range(3)]
    #     new_p = self.genome[0:4] if selections[0] else other_parent.genome[0:4]
    #     new_q = self.genome[4:8] if selections[1] else other_parent.genome[4:8]
    #     new_r = self.genome[8:12] if selections[2] else other_parent.genome[8:12]
    #     new_genome = new_p + new_q + new_r
    #     # make the new child and mutate it.
    #     child = Agent(id=next_id[0], genome=new_genome, gen_num=self.gen_num+1)
    #     next_id[0] += 1
    #     child.mutate()
    #     return child

    # write this agent to the file.
    def write_to_file(self, filepath:str):
        # don't overwrite with each statement (a=append).
        file1 = open(filepath, "a+")
        if stat(filepath).st_size == 0:
            # if this is a new file, we have a problem.
            raise Exception ("File does not exist for " + filepath)
        # add the row for this round of results
        row = ",".join([str(self.id),str(self.gen_num),str(self.fitness)] + [str(g) for g in self.genome])
        file1.write(row + "\n")
        file1.close()

    # set the KF to use this agent's genome.
    def set_genome(self):
        filepath = "config/genome.csv"
        file1 = open(filepath, "w")
        row = ",".join([str(item) for item in self.genome])
        file1.write(row)
        file1.close
    
    # read the KF data to calculate a fitness for this agent.
    def calc_fitness(self, fpath:str):
        # fitness will be a weighted sum of the difference 
        # between the KF state and the truth.
        file1 = open(fpath + ".csv", "r")
        # skip the header (first line)
        file1.seek(0,1)
        tot_fit = 0
        num_timesteps = 0
        for line in file1.readlines():
            l = line.split(",")
            tot_fit += abs(float(l[8])-float(l[12])) + abs(float(l[9])-float(l[13]))
            num_timesteps += 1
        file1.close()
        self.fitness = tot_fit / num_timesteps