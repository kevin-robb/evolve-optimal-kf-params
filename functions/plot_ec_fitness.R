# Plot agents' fitness by generation to show evolution.
#
# Generates a PNG showing a column of all agents in a generation.
#
plot_ec_fitness <- function(filename, png=TRUE) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  library(cowplot)
  
  # read in the data from the file. first line is header.
  filepath = paste("./ec_data/", filename, ".csv", sep="")
  df=read.csv(filepath)

  #head(df)
  
  # define the plot. use only generation # and fitness.
  #df_subset = melt(df[,c(1,14)], id=c("generation_number"))
  p <- ggplot(df) + geom_point(aes(generation_number,fitness),color="blue") +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("Fitness Distribution For Each Generation") + 
    cowplot::theme_minimal_grid(12) +
    theme(axis.text.x = element_text(size = 8, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12)) + 
    theme(legend.position="none") + 
    ylab("Fitness (Lower is Better)") + xlab("Generation") +
    ylim(c(0,max(df->fitness))) +
    # set x axis labels to integers only
    scale_x_continuous(breaks=c(1,2,3,4,5))
  
  # write the plot to a file.
  if (png == TRUE) {
    plot_path = paste("./ec_plots/", filename, ".png", sep="")
    cowplot::save_plot(plot_path,p,base_height=4,base_width=6.5)
  } else {
    p
  }
  
}