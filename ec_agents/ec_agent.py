# this file defines our evolutionary computation agents
from typing import List, Tuple
from random import random, uniform, randint

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
    def __init__(self, genome:List[float] = None):
        self.fitness = -1
        if genome is not None:
            self.genome = genome
        else:
            self.genome = self.genome_init
    
    # mutate the genome of this agent.
    def mutate(self):
        # choose a gene to mutate.
        gene = randint(0,len(self.genome))
        # choose whether this mutation will be pos/neg.
        sign = -1 if randint(0,1) < 1 else 1
        # choose the amount to mutate (not more than 100% the current value)
        change = min(random() * self.genome[gene], 0.005)
        # make the mutation.
        self.genome[gene] += sign * change

    # cross the genes of this agent with another to produce a child.
    def crossover(self, other_parent:"Agent") -> "Agent":
        # decide which genes will be taken from which parent.
        new_genome = [self.genome[gene] if randint(0,1) < 1 else other_parent.genome[gene] for gene in range(len(self.genome))]
        child = Agent(new_genome)
        child.mutate()
        return child
