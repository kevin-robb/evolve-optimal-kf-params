## Plot KF data with track from command line. format:
# Rscript --vanilla functions/plot_cl_track.R "filename" true

#!/usr/bin/env Rscript
# grab parameters from command line
args = commandArgs(trailingOnly=TRUE)

# test if there is at least one argument: if not, return an error
if (length(args)==0) {
  stop("Must supply filename (without extension) and optionally the directory", call.=FALSE)
} else if (length(args)==1) {
  # run the script as previously, with one file in the kf_data directory.
  args[2] = "kf_data"
}

# source the function
source("functions/plot_with_track.R")

## Call function with command line params
plot_with_track(filename=args[1],dirpath=args[2])

