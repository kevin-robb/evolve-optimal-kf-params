## Plot EC fitness data from command line. format:
# Rscript --vanilla functions/plot_cl_fitness.R "filename" true

#!/usr/bin/env Rscript
# grab parameters from command line
args = commandArgs(trailingOnly=TRUE)

# test if there is at least one argument: if not, return an error
if (length(args)==0) {
  stop("Must supply run directory in format 'runs/run_2021-03-08-18-23-03'", call.=FALSE)
} else if (length(args)==1) {
  # when called from the command line, by default write to PNG
  args[2] = "true"
}

# source the function
source("functions/plot_ec_fitness.R")

## Call function with command line params
# keep track of whether to write to PNG or not
if (tolower(args[2]) == "true") {
  plot_ec_fitness(dirpath=args[1],png=TRUE)
} else {
  plot_ec_fitness(dirpath=args[1],png=FALSE)
}
