# Plot 6-Dimensional Kalman Filter Data.
#
# Generates a PNG graphing all six KF parameters' 
# measured, predicted, and state values.
#
# Also plot the ground truth for x and y position.
#
plot_with_track <- function(filename, png=FALSE, w=1000,h=750) {
  library(reshape2)
  library(ggplot2)
  library(grid)
  library(gridExtra)
  library(cowplot)
  
  # read in the data from the file
  filepath = paste("./data/", filename, ".csv", sep="")
  df=read.csv(filepath)
  names(df) <- c("x_meas","y_meas","xdot_meas","ydot_meas","theta_meas","yaw_rate_meas","x_pred","y_pred","xdot_pred","ydot_pred","theta_pred","yaw_rate_pred","x","y","xdot","ydot","theta","yaw_rate","x_true","y_true","vel_true")
  # add a timestep independent variable
  t = c(1:length(df$x_meas))
  df <- cbind(t, df)
  #head(df)
  
  # subset each variable
  df_x = melt(df[,c(1,2,8,14,20)], id=c("t"))
  df_y = melt(df[,c(1,3,9,15,21)], id=c("t"))
  df_xdot = melt(df[,c(1,4,10,16)], id=c("t"))
  df_ydot = melt(df[,c(1,5,11,17)], id=c("t"))
  df_theta = melt(df[,c(1,6,12,18)], id=c("t"))
  df_yr = melt(df[,c(1,7,13,19)], id=c("t"))
  
  # define all the plots
  p_x <- ggplot(df_x) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("X Position") + 
    cowplot::theme_minimal_grid(12) + theme(legend.position="none")
  p_y <- ggplot(df_y) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green","black")) +
    ggtitle("Y Position") + 
    cowplot::theme_minimal_grid(12) + theme(legend.position="none")
  p_xdot <- ggplot(df_xdot) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("X Velocity") + theme_minimal_grid(12) + theme(legend.position="none")
  p_ydot <- ggplot(df_ydot) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Y Velocity") + theme_minimal_grid(12) + theme(legend.position="none")
  p_theta <- ggplot(df_theta) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Heading") + theme_minimal_grid(12) + theme(legend.position="none")
  p_yr <- ggplot(df_yr) + geom_line(aes(x=t,y=value,colour=variable)) +
    scale_colour_manual(values=c("red","blue","green")) +
    ggtitle("Yaw Rate") + theme_minimal_grid(12) + theme(legend.position="none")
  
  # draw the robot's path (measured and true)
  df_t = df[,c(1,2,8,14,20,3,9,15,21)]
  #names(df_t) <- c("t","x_meas","x_pred","x","x,true","y_meas","y_pred","y","y_true")
  p_t <- ggplot(df_t) +
    ggtitle("Robot Path (X vs Y)") +
    xlab("Y Position") + ylab("X Position") +
    geom_point(aes(y_meas,x_meas,colour="red")) +
    geom_point(aes(y_pred,x_pred,colour="blue")) +
    geom_point(aes(y,x,colour="green")) +
    geom_point(aes(y_true,x_true,colour="black")) +
    # scale_shape_discrete(name="Variable Shown",
    #                     breaks=c("red","blue","green","black"),
    #                     labels=c("Measured","Predicted","State","True")) +
    theme_minimal_grid(12) + theme(legend.position="none")
  
  # plot them using cowplot for alignment & layout
  #grid.arrange(p_x,p_y,p_xdot,p_ydot,p_theta,p_yr, nrow=3)
  ptl <- cowplot::align_plots(p_x,p_xdot,align='v',axis='lr')
  ptr <- cowplot::align_plots(p_y,p_ydot,align='v',axis='lr')
  pl <- cowplot::plot_grid(ptl[[1]], ptr[[1]], ncol=2)
  pr <- cowplot::plot_grid(ptl[[2]],ptr[[2]], ncol=2)
  
  plb <- cowplot::align_plots(pl,p_theta,align='v',axis='lr')
  prb <- cowplot::align_plots(pr,p_yr,align='v',axis='lr')
  pb <- cowplot::plot_grid(plb[[2]],prb[[2]],ncol=2)
  
  p6 <- cowplot::plot_grid(pl,pr,pb,ncol=1) #main 6
  
  p_tot <- cowplot::plot_grid(p6, p_t, ncol=2) #add track
  p_tot
  
  if (png == TRUE) {
    plot_path = paste("./plots/", filename, "_track", ".png", sep="")
    png(file=plot_path,width=w,height=h)
    save_plot(plot_path,p_tot)
  }
  
}