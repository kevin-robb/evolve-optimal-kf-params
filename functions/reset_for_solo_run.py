# This simple function resets config info for the KF to run a single time on its own.
# Basically, this clears remnants of previous EC runs.

from datetime import datetime

def set_genome_to_default():
    # reset the genome to its default values.
    p_diag = [0.25, 0.25, 0.25, 0.5]
    q_diag = [0.01, 0.01, 0.01, 0.01]
    r_diag = [0.01, 0.01, 0.01, 0.01]
    genome = p_diag + q_diag + r_diag
    filepath = "config/genome.csv"
    file1 = open(filepath, "w")
    row = ",".join([str(item) for item in genome])
    file1.write(row)
    file1.close

def set_filepath_to_default() -> str:
    # set the KF data config to output to the default destination.
    filepath = "config/kf_data_destination.csv"
    file1 = open(filepath, "w")
    # make the filename to return
    dt = datetime.now()
    fname = "kf_" + dt.strftime("%Y-%m-%d-%H-%M-%S")
    row = "kf_data/" + fname
    #row = "kf_data/,default"
    file1.write(row)
    file1.close
    return fname

def main():
    set_genome_to_default()
    return set_filepath_to_default()

if __name__ == "__main__":
    main()