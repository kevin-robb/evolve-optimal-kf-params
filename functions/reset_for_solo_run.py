# This simple function resets config info for the KF to run a single time on its own.
# Basically, this clears remnants of previous EC runs.

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

def set_filepath_to_default():
    # set the KF data config to output to the default destination.
    filepath = "config/kf_data_destination.csv"
    file1 = open(filepath, "w")
    row = "kf_data/, default"
    file1.write(row)
    file1.close

def main():
    set_genome_to_default()
    set_filepath_to_default()




if __name__ == "__main__":
    main()