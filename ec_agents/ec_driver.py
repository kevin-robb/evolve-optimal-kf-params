# This file creates and manages the set of agents, and assigns fitness.

import read_results
from ec_agent import Agent
from typing import Dict, List

def initialize_agents(roster_size:int) -> List:
    roster = []
    for i in range(roster_size):
        roster.append(Agent())
    return roster

def assign_fitness(roster:List, index:int):
    results = read_results.read_file()
    roster[index].fitness = results["Score"]

def next_generation(roster:List):
    # sort this generation by fitness.
    #best_fitness = max([a.fitness for a in roster])
    roster.sort(key=lambda a: a.fitness, reverse=True)
    # for now just copy top half and do crossovers to fill the gaps.
    # TODO look into better ways to select for reproduction.
    
