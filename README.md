# capstone-kf-ml
My engineering physics capstone work regarding Kalman filters and machine learning.

# Launch instructions
There are two ways to run this project. I have written scripts to streamline the many steps and coalesce both use cases as much as possible. In either case, you must first source the ROS workspace with the command `source sim_ws/devel/setup.bash`.

 - To run only the ROS node with the KF using the default settings, run `bash run_once.sh`. This will generate data for the KF in the `kf_data` directory, and automatically create a plot in the `kf_plots` directory. Both will have the same unique filename created using the current datetime. 
 - To initiate the evolutionary computation aspect of this project, run `python3 ec_driver.py`. After entering this command, you will be prompted for the number of agents to use in each generation, followed by the number of generations to run, and lastly asked whether agents should be initialized randomly or mutated from the default. It will then proceed to seed the KF with each agent's genome, run the simulator for it, calculate fitness, and repeat for all agents. After all agents have been run, the next generation will be formed wih crossover/mutation, and the process will repeat for all generations. A directory will be created within `runs/` corresponding to the current datetime, and data/plots for every agent as well as summary data/plots will be saved in it.

(Everything should happen automatically after the initial command. The `ec_driver.py` script will repeatedly run `run_once.sh` to run the simulation for each agent. This takes a while, and can't really be run in the background since the simulator will repeatedly close and reopen. Output from ROS is suppressed to prevent the console from filling with red text, since roscore is killed when the simulator finishes a run.)

# Modifying parameters of this project
If you'd like to experiment with the parameters of this project, there are several aspects that could be easily modified to see how they affect the results. Feel free to fork this repository and play around with it. Below are some areas that would allow for such level of experimentation. 

 - `ec_driver.py`: Selection process for reproduction
 - `ec_agent.py`: Crossover procedure, mutation procedure, aggressiveness of both
 - `config/default_genome.csv`: Default KF genome values, and can add additional genome parameters
 - `sim_ws/src/capstone/src/kf.py`: KF properties and structure
 - `sim_ws/src/swc_localization/src/localization_node.py`: Policy for choosing next destination
 - `sim_ws/src/swc_control/src/control_node.py`: Motion policy, obstacle avoidance (both could definitely use improvement)

