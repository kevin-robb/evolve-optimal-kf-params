## Plot KF data with track from command line. format:
# Rscript --vanilla functions/plot_cl_track.R "filename" true

#!/usr/bin/env Rscript
# grab parameters from command line
args = commandArgs(trailingOnly=TRUE)

# test if there is at least one argument: if not, return an error
if (length(args)==0) {
  stop("Must supply filename in format 'kf_o0_n2_2020-11-15-23-19-13'", call.=FALSE)
} else if (length(args)==1) {
  # when called from the command line, by default write to PNG
  #args[2] = "out.txt"
  args[2] = "true"
}

# source the function
source("functions/plot_with_track.R")

## Call function with command line params
# keep track of whether to write to PNG or not
if (tolower(args[2]) == "true") {
  plot_with_track(filename=args[1],png=TRUE)
} else {
  plot_with_track(filename=args[1],png=FALSE)
}
