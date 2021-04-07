#!/usr/bin/env Rscript

# Plot 4-Dimensional Kalman Filter Data from a single run.
#
# Generates a PNG graphing all four KF parameters' 
# measured, predicted, and state values, as well
# as the ground truth for comparison.
#
# Also shows the path followed by the robot (X vs Y),
# and optionally show the measured heading in a separate plot.
#
plot_one_run <- function(filename, dirpath, plot_hdg=FALSE) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  library(cowplot)

  # get the waypoints (TODO this was causing errors, need to fix)
  # wp_path = "./config/waypoints.csv"
  # wp_df=read.csv(wp_path)
  
  # read in the data from the file
  filepath = paste("./", dirpath, "/", filename, ".csv", sep="")
  df=read.csv(filepath)

  # data file has 18 columns, (will be 19 when timestep is added)
  # meas (4d), pred (4d), state (4d), truth (5d), cur_hdg (1d)
  meas_names <- c("x_meas","y_meas","xdot_meas","ydot_meas")
  pred_names <- c("x_pred","y_pred","xdot_pred","ydot_pred")
  state_names <- c("x_state","y_state","xdot_state","ydot_state")
  truth_names <- c("x_true","y_true","xdot_true","ydot_true","vel_true")
  yaw_name <- c("cur_hdg")
  names(df) <- c(meas_names, pred_names, state_names, truth_names, yaw_name)

  # add a timestep independent variable
  t = c(1:length(df$x_meas))
  df <- cbind(t, df)
  
  #head(df)
  
  # change the names of df_x since they will be used for the legend
  df_x_pre <- df[,c(1,2,6,10,14)]
  names(df_x_pre) <- c("t","Measured","Predicted","State","Truth")
  
  # subset each variable
  df_x = melt(df_x_pre, id=c("t"))
  df_y = melt(df[,c(1,3,7,11,15)], id=c("t"))
  df_xdot = melt(df[,c(1,4,8,12,16)], id=c("t"))
  df_ydot = melt(df[,c(1,5,9,13,17)], id=c("t"))
  
  # define the first plot (x-position)
  p_x <- ggplot(df_x) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("X Position") + 
    cowplot::theme_minimal_grid(12) +
    theme(axis.text.x = element_text(size = 8, angle = 90, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12))
  # save the legend before suppressing it
  legend <- cowplot::get_legend(p_x)
  p_x <- p_x + theme(legend.position="none") + ylab("") + xlab("timestep")
  
  # define all the other plots (w/o legend)
  p_y <- ggplot(df_y) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("Y Position") + ylab("") + xlab("timestep") +
    cowplot::theme_minimal_grid(12) + theme(legend.position="none") +
    theme(axis.text.x = element_text(size = 8, angle = 90, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12))
  p_xdot <- ggplot(df_xdot) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green","black")) + ylab("") + xlab("timestep") +
    ggtitle("X Velocity") + theme_minimal_grid(12) + theme(legend.position="none") +
    theme(axis.text.x = element_text(size = 8, angle = 90, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12))
  p_ydot <- ggplot(df_ydot) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green","black")) + ylab("") + xlab("timestep") +
    ggtitle("Y Velocity") + theme_minimal_grid(12) + theme(legend.position="none") +
    theme(axis.text.x = element_text(size = 8, angle = 90, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
    theme(plot.title = element_text(size=12))
  
  # draw the robot's path
  p_t <- ggplot(df) +
    scale_x_reverse() +
    ggtitle("Robot Path (X vs Y)") +
    xlab("Y Position") + ylab("X Position") +
    geom_point(aes(y_true,x_true), color="black") +
    geom_point(aes(y_meas,x_meas), color="red") +
    geom_point(aes(y_pred,x_pred),color="blue") +
    geom_point(aes(y_state,x_state),color="green") +
    theme(plot.title = element_text(size=14)) +
    coord_cartesian(xlim = c(25, -25), ylim = c(0, 90)) +
    theme_minimal_grid(12) #+ theme(legend.position="none")
  p_t

  # TODO add the waypoints to the path plot
  
  ## combine plots

  # main 4 plots (x,y,xdot,ydot) will be left side
  ptl <- cowplot::align_plots(p_x,p_xdot,align='v')
  ptr <- cowplot::align_plots(p_y,p_ydot,align='v')
  pl <- cowplot::plot_grid(ptl[[1]], ptr[[1]], ncol=2)
  pr <- cowplot::plot_grid(ptl[[2]],ptr[[2]], ncol=2)
  p_left <- cowplot::plot_grid(pl,pr,ncol=1)
  
  # track and legend will be right side
  p_right <- cowplot::plot_grid(p_t, legend, ncol=2, rel_widths=c(2,1))
  
  if (plot_hdg == TRUE) {
    # make the heading plot
    p_hdg <- ggplot(df) + geom_line(aes(x=t,y=cur_hdg),color="red") + ylab("") + xlab("timestep") +
      ggtitle("Heading") + theme_minimal_grid(12) + theme(legend.position="none") +
      theme(axis.text.x = element_text(size = 8, angle = 90, vjust = 0.5),axis.text.y = element_text(size = 8, vjust = 0.5)) +
      theme(plot.title = element_text(size=12))
    # put it in the right side plot grid
    # right side will be track+legend above heading plot
    p_right <- cowplot::plot_grid(p_right,p_hdg,ncol=1,rel_heights=c(2,1))
  }
  
  # make overall plot
  p_tot <- cowplot::plot_grid(p_left,p_right,ncol=2,rel_widths=c(4,3))
  p_tot
  
  # set plot location
  if (dirpath == "kf_data") {
    # default option
    plot_path = paste("./kf_plots/", filename, ".png", sep="")
  } else {
    # write the plots to the same file the data came from
    plot_path = paste("./", dirpath, "/", filename, ".png", sep="")
  }
  # store the plot PNG
  cowplot::save_plot(plot_path,p_tot,base_height=4,base_width=6.5)
  
}


## Plot KF data with track from command line. format:
# Rscript --vanilla functions/plot_one_run.R "filename" true

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
#source("functions/plot_one_run.R")

## Call function with command line params
plot_one_run(filename=args[1],dirpath=args[2])