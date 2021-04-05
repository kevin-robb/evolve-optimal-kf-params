# Plot agents' fitness by generation to show evolution.
#
# Generates a PNG showing a column of all agents in a generation.
#
plot_ec_fitness <- function(dirpath, png=TRUE) {
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
  
  # define the plot. use only generation # and fitness.
  p <- ggplot(df) + geom_point(aes(as.numeric(generation_number),as.numeric(fitness)),color="blue") +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("Fitness Distribution For Each Generation") + 
    cowplot::theme_minimal_grid(12) +
    theme(axis.text.x = element_text(size = 8, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12)) + 
    theme(legend.position="none") + 
    ylab("Fitness (Lower is Better)") + xlab("Generation") +
    ylim(c(0,max(df->fitness))) + #ylim(c(0,500)) + 
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