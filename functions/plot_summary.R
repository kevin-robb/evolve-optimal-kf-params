#!/usr/bin/env Rscript

# To use from cmd line, invoke:
# Rscript --vanilla functions/plot_summary.R "directory name"
# To use in another R/Rmd script:
# source("functions/plot_summary.R")

# Plot agents' cost function (fitness) by generation to show evolution.
#
# Generates a PNG showing a boxplot for each generation.
#
plot_summary <- function(dirpath, png=TRUE) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  library(cowplot)
  
  # read in the data from the file. first line is header.
  filepath = paste("./", dirpath, "/summary.csv", sep="")
  df=read.csv(filepath, header=TRUE)
  # determine how many agents are in each gen, to use in title.
  agents_per_gen = max(df[df$generation_number == 1,]$agent_id)
  # change gen num to categorical
  df$generation_number <- as.factor(df$generation_number)
  # remove rows that resulted in error
  df = df[df$fitness != 800.0, ]
  # remove rows that somehow have 0 fitness TODO figure out how this happens
  df = df[df$fitness != 0.0, ]

  # make a boxplot
  p <- ggplot(df, aes(generation_number, fitness)) + 
    geom_boxplot(fill="skyblue",outlier.colour = "red",outlier.shape = 1)
  # add all the plot formatting.
  p <- p +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle(paste("Cost Distributions (with", agents_per_gen, "Agents/Generation)", "")) + 
    cowplot::theme_minimal_grid(12) +
    theme(axis.text.x = element_text(size = 8, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12)) + 
    theme(legend.position="none") + 
    ylab("Mean Difference from Truth (m)") + xlab("Generation")

  # write the plot to a file.
  if (png == TRUE) {
    fname = "summary.png"
    plot_path = paste("./", dirpath, "/", fname, sep="")
    cowplot::save_plot(plot_path,p,base_height=3,base_width=4.5)
  } else {
    p
  }
}

## Command Line stuff

# grab parameters from command line
args = commandArgs(trailingOnly=TRUE)
# check number of arguments.
if (length(args)==1) {
  # create the plot
  plot_summary(dirpath=args[1])
} else if (length(args)>1) {
  stop("Too many command line arguments", call.=FALSE)
} else { #length(args)==0
  # Comment out this line to be able to source this file
  # and use the function from somewhere besides the cmd line.
  stop("Must supply run directory in format 'runs/run_2021-03-08-18-23-03'", call.=FALSE)
}
