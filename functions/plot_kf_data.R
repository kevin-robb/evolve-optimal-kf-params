# Plot Kalman Filter Data
#
# Generates a PNG graphing all six KF parameters' 
# measured, predicted, and state values.
#
plot_kf_data <- function(filename, png=FALSE, w=1000,h=750) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  
  # read in the data from the file
  #filename = "kf_data_2020-11-14-18-52-24"
  filepath = paste("./data/", filename,".csv", sep="")
  df=read.csv(filepath)
  names(df) <- c("x_meas","y_meas","xdot_meas","ydot_meas","theta_meas","yaw_rate_meas","x_pred","y_pred","xdot_pred","ydot_pred","theta_pred","yaw_rate_pred","x","y","xdot","ydot","theta","yaw_rate")
  # add a timestep independent variable
  t = c(1:length(df$x_meas))
  df <- cbind(t, df)
  head(df)
  
  if (png == TRUE){
    # open the file for plotting
    plot_path = paste("./plots/", filename, ".png", sep="")
    png(file=plot_path,width=w,height=h)
  }
  
  # subset each variable
  df_x = melt(df[,c(1,2,8,14)], id=c("t"))
  df_y = melt(df[,c(1,3,8,15)], id=c("t"))
  df_xdot = melt(df[,c(1,4,10,16)], id=c("t"))
  df_ydot = melt(df[,c(1,5,11,17)], id=c("t"))
  df_theta = melt(df[,c(1,6,12,18)], id=c("t"))
  df_yr = melt(df[,c(1,7,13,19)], id=c("t"))
  
  # setup layout
  layout <- matrix(seq(1, 6), ncol = 2, nrow = 3)
  
  # define all the plots
  p_x <- ggplot(df_x) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("X Position")
  p_y <- ggplot(df_y) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Y Position")
  p_xdot <- ggplot(df_xdot) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("X Velocity")
  p_ydot <- ggplot(df_ydot) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Y Velocity")
  p_theta <- ggplot(df_theta) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Heading")
  p_yr <- ggplot(df_yr) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Yaw Rate")
  
  # plot them
  grid.arrange(p_x,p_y,p_xdot,p_ydot,p_theta,p_yr, nrow=3)
  
  if (png == TRUE) {
    # close the file
    dev.off()
  }
  
}