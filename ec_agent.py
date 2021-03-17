# this file defines our evolutionary computation agents
from typing import List, Tuple
from random import random, uniform, randint, choice
from os import stat
from numpy.random import normal

class Agent:
    # Genome:
    # - diagonal entries of P_init (covariance)
    # - diagonal entries of Q (process noise)
    # - diagonal entries of R (measurement uncertainty)
    p_diag = [0.25, 0.25, 0.25, 0.5]
    q_diag = [0.01, 0.01, 0.01, 0.01]
    r_diag = [0.01, 0.01, 0.01, 0.01]
    genome_init = p_diag + q_diag + r_diag

    # instantiate an agent, either with the default genome or a given one.
    def __init__(self, id:int, genome:List[float] = None, gen_num:int = 1):
        self.id = id
        self.fitness = -1
        self.results = {}
        self.gen_num = gen_num
        if genome is not None:
            self.genome = genome
        else:
            self.genome = self.genome_init
            self.randomize_genome()
    
    # randomize the genome for the first generation.
    def randomize_genome(self):
        # use a gaussian w/ mean 0 and std dev 0.1 to change each gene.
        self.genome = [min(abs(g + normal(loc=0, scale=0.01)), 0.005) for g in self.genome]

    # mutate the genome of this agent.
    def mutate(self):
        # choose a gene to mutate.
        gene = randint(0,len(self.genome))
        # choose the amount to mutate (gaussian, mean 0, std dev 0.1) and do it.
        # make sure it is positive, and that the value doesn't drop too close to 0.
        self.genome[gene] = min(abs(self.genome[gene] + normal(loc=0, scale=0.01)), 0.005)

    # cross the genes of this agent with another to produce a child.
    def crossover(self, other_parent:"Agent", next_id:List[int]) -> "Agent":
        # decide which genes will be taken from which parent.
        new_genome = [self.genome[gene] if randint(0,1) < 1 else other_parent.genome[gene] for gene in range(len(self.genome))]
        child = Agent(new_genome, next_id[0], self.gen_num+1)
        next_id[0] += 1
        child.mutate()
        return child
    
    # perform crossover, but do not separate groups of genes belonging to the same matrix.
    def group_crossover(self, other_parent:"Agent", next_id:List[int]) -> "Agent":
        # arbitrarily take gene groups from one parent each.
        selections = [choice([True, False]) for i in range(3)]
        new_p = self.genome[0:4] if selections[0] else other_parent.genome[0:4]
        new_q = self.genome[4:8] if selections[1] else other_parent.genome[4:8]
        new_r = self.genome[8:12] if selections[2] else other_parent.genome[8:12]
        new_genome = new_p + new_q + new_r
        # make the new child and mutate it.
        child = Agent(new_genome, next_id[0], self.gen_num+1)
        next_id[0] += 1
        child.mutate()
        return child

    # write this agent to the file.
    def write_to_file(self, filepath:str):
        # don't overwrite with each statement (a=append).
        file1 = open(filepath, "a+")
        if stat(filepath).st_size == 0:
            # if this is a new file, we have a problem.
            raise Exception ("File does not exist for " + filepath)
        # add the row for this round of results
        row = ",".join([str(self.gen_num)] + [str(g) for g in self.genome] + [str(self.fitness)])
        file1.write(row + "\n")
        file1.close()

    # set the KF to use this agent's genome
    def set_genome(self):
        filepath = "sim_ws/src/capstone/src/genome.csv"
        file1 = open(filepath, "w")
        row = ",".join([str(item) for item in self.genome])
        file1.write(row)
        file1.close