#!/usr/bin/env Rscript

# Plot agents' fitness by generation to show evolution.
#
# Generates a PNG showing a column of all agents in a generation (full=TRUE)
# or only the best and median (full=FALSE).
#
plot_summary <- function(dirpath, png=TRUE, full=TRUE) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  library(cowplot)
  
  # read in the data from the file. first line is header.
  filepath = paste("./", dirpath, "/summary.csv", sep="")
  df=read.csv(filepath, header=TRUE)

  # the highest gen_num will be the max on the x-axis.
  max_gen = tail(df$generation_number)

  if (full) {
    p <- ggplot(df) + 
      geom_point(aes(as.numeric(generation_number),as.numeric(fitness)),color="blue") +
      ylim(c(0,max(df->fitness)))
  } else {
    # if we are NOT running with full=TRUE, extract max and median in each gen.
    require(data.table)
    group <- as.data.table(df)
    gen_best=group[group[, .I[which.max(fitness)], by=generation_number]$V1]
    gen_medians=group[group[, .I[which.median(fitness)], by=generation_number]$V1]
  }
  
  
  # define the plot. use only generation # and fitness.
  p <- p +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("Fitness Distribution For Each Generation") + 
    cowplot::theme_minimal_grid(12) +
    theme(axis.text.x = element_text(size = 8, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12)) + 
    theme(legend.position="none") + 
    ylab("Fitness (Lower is Better)") + xlab("Generation") +
    # set x axis labels to integers only
    scale_x_continuous(breaks=integer_breaks(max_gen))
  
  # write the plot to a file.
  if (png == TRUE) {
    plot_path = paste("./", dirpath, "/summary.png", sep="")
    cowplot::save_plot(plot_path,p,base_height=4,base_width=6.5)
  } else {
    p
  }
  
}

# A function factory for getting integer y-axis values.
# https://www.r-bloggers.com/2019/11/setting-axes-to-integer-values-in-ggplot2/
integer_breaks <- function(n = 5, ...) {
fxn <- function(x) {
breaks <- floor(pretty(x, n, ...))
names(breaks) <- attr(breaks, "labels")
breaks
}
return(fxn)
}



## Plot EC fitness data from command line. format:
# Rscript --vanilla functions/plot_summary.R "filename" true

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
#source("functions/plot_summary.R")

## Call function with command line params
# keep track of whether to write to PNG or not
if (tolower(args[2]) == "true") {
  plot_summary(dirpath=args[1],png=TRUE)
} else {
  plot_summary(dirpath=args[1],png=FALSE)
}