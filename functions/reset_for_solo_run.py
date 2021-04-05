# This simple function resets config info for the KF to run a single time on its own.
# Basically, this clears remnants of previous EC runs.

from datetime import datetime
from os import putenv
import sys

def set_genome_to_default():
    # reset the genome to its default values.
    filepath = "config/default_genome.csv"
    file3 = open(filepath, "r")
    genome = [float(g) for g in file3.readlines()[1].split(",")]
    file3.close()

    # p_diag = [0.25, 0.25, 0.25, 0.5]
    # q_diag = [0.01, 0.01, 0.01, 0.01]
    # r_diag = [0.01, 0.01, 0.01, 0.01]
    # control_params = [0.5, 5.0, 0.3]
    # genome = p_diag + q_diag + r_diag + control_params
    filepath = "config/genome.csv"
    file1 = open(filepath, "w")
    row = ",".join([str(item) for item in genome])
    file1.write(row)
    file1.close

def set_filepath_to_default(fname:str):
    # set the KF data config to output to the default destination.
    filepath = "config/kf_data_destination.csv"
    file1 = open(filepath, "w")
    # make the filename to return.
    #dt = datetime.now()
    #fname = "kf_" + dt.strftime("%Y-%m-%d-%H-%M-%S")
    row = "kf_data/" + fname
    file1.write(row)
    file1.close

def main():
    # receive the filename to use.
    fname = sys.argv[1]
    set_genome_to_default()
    set_filepath_to_default(fname)
    # let the bash script know the filename so it can plot the data.
    #putenv("FNAME", fname)

if __name__ == "__main__":
    main()