#!/usr/bin/env Rscript

# To use from cmd line, invoke:
# Rscript --vanilla functions/plot_summary.R "directory name"
# To use in another R/Rmd script:
# source("functions/plot_summary.R")

# Plot agents' fitness by generation to show evolution.
#
# Generates a PNG showing a column of all agents in a generation (full=TRUE)
# or only the best and median (full=FALSE).
#
plot_summary <- function(dirpath, full=TRUE, png=TRUE) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  library(cowplot)
  library(arules)
  
  # read in the data from the file. first line is header.
  filepath = paste("./", dirpath, "/summary.csv", sep="")
  df=read.csv(filepath, header=TRUE)

  # the highest gen_num will be the max on the x-axis.
  max_gen = max(df$generation_number)
  # change gen num to categorical
  df$generation_number <- as.factor(df$generation_number)

  # remove rows that resulted in error
  df = df[df$fitness != 800.0, ]
  # remove rows that somehow have 0 fitness TODO figure out how this happens
  df = df[df$fitness != 0.0, ]

  # subset the dataframe into only the stuff we care about.
  #df=data.frame(df$generation_number,df$fitness)

  # # extract max and median in each gen.
  # df_quart <- data.frame(generation_number=integer(), best=double(), median=double(), stringsAsFactors=FALSE)
  # # track overall max for plot limits.
  # max <- 0
  # for (gen in seq(1,max_gen,1)) {
  #   # only look at agents in this generation.
  #   df_sub <- subset(df, generation_number==gen)
  #   # find the best (min) & median values.
  #   best <- min(df_sub$fitness)
  #   med <- median(df_sub$fitness)
  #   # update the overall max.
  #   max <- max(c(max, med))
  #   # create df with new values.
  #   df_temp <- data.frame(gen,best,med)
  #   names(df_temp) <- c("generation_number","best","median")
  #   # add this row to our summary df.
  #   df_quart <- rbind(df_quart, df_temp)
  # }
  # # now we have the data we want isolated in df_quart. plot it.
  # df_quart <- melt(df_quart ,  id.vars = 'generation_number', variable.name = 'quantity')
  
  # # use a different base depending if this is a full plot.
  # if (full) {
  #   p <- ggplot(df) + 
  #     geom_point(aes(as.numeric(generation_number),as.numeric(fitness)),color="blue") +
  #     ylim(c(0,max(df$fitness)))
  #   p <- p + geom_point(aes(x=df_quart$generation_number, y=df_quart$median, colour="green"),data=df_quart) +
  #     geom_point(aes(x=df_quart$generation_number, y=df_quart$best, colour="red"),data=df_quart)
  # } else {
  #   p <- ggplot(df_quart, aes(generation_number, value)) +
  #     geom_point(aes(colour = quantity)) + ylim(c(0,max))
  # }

  # add a duplicate of gen num and then discretize it.
  # df$gen_num_discr = df$generation_number
  # df$gen_num_discr <- discretize(df$gen_num_discr, method="fixed",breaks=seq(1,max_gen,1))#breaks=seq(1,max_gen,1),labels=seq(1,max_gen+1,1))

  # make a boxplot
  p <- ggplot(df, aes(generation_number, fitness)) + geom_boxplot(fill="skyblue",outlier.colour = "red",outlier.shape = 1)
  
  # add all the plot formatting.
  p <- p +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("Cost Distribution For Each Generation") + 
    cowplot::theme_minimal_grid(12) +
    theme(axis.text.x = element_text(size = 8, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12)) + 
    theme(legend.position="none") + 
    ylab("Mean Difference from Truth") + xlab("Generation") #+
    # set x axis labels to integers only.
    #scale_x_continuous(breaks=integer_breaks(max_gen))
    #scale_x_discrete(breaks = function(x) unique(floor(pretty(seq(0, (max(x) + 1) * 1.1)))))
  
  # write the plot to a file.
  if (png == TRUE) {
    if (full) {
      fname = "summary_full.png"
    } else {
      fname = "summary.png"
    }
    plot_path = paste("./", dirpath, "/", fname, sep="")
    cowplot::save_plot(plot_path,p,base_height=3,base_width=4.5)
  } else {
    p
  }
}

# A function for getting integer axis values.
# https://www.r-bloggers.com/2019/11/setting-axes-to-integer-values-in-ggplot2/
integer_breaks <- function(n = 5, ...) {
  fxn <- function(x) {
    breaks <- floor(pretty(x, n, ...))
    names(breaks) <- attr(breaks, "labels")
    breaks
  }
  return(fxn)
}

## Command Line stuff

# grab parameters from command line
args = commandArgs(trailingOnly=TRUE)

# check number of arguments.
if (length(args)==1) {
  # create both a full plot and condensed plot.
  #plot_summary(dirpath=args[1],full=TRUE)
  plot_summary(dirpath=args[1],full=FALSE)
} else if (length(args)>1) {
  stop("Too many command line arguments", call.=FALSE)
} else { #length(args)==0
  # Comment out this line to be able to source this file
  # and use the function from somewhere besides the cmd line.
  stop("Must supply run directory in format 'runs/run_2021-03-08-18-23-03'", call.=FALSE)
}
