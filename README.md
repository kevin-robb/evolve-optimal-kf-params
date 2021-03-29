# capstone-kf-ml
My engineering physics capstone work regarding Kalman filters and machine learning.

# Launch instructions
There are two ways to run this project. I have written scripts to streamline the many steps and coalesce both use cases as much as possible.

 - To run only the ROS node with the KF using the default settings, run `bash run_once.sh default`. This will generate data for the KF in the `kf_data` directory, and automatically create a plot in the `kf_plots` directory. Both will have the same unique filename created using the current datetime. 
 - To initiate the evolutionary computation aspect of this project, run `python3 ec_driver.py`. This will create random genomes for several agents in the first generation, and run the simulation for each agent with the genome seeding the KF parameters. It will then calculate a fitness and perform crossover/mutation to form the next generation. All data and plots associated with this run will be stored in a unique directory within `runs/` that is created with the current datetime.

Everything should happen automatically after the initial command. The `ec_driver.py` script will repeatedly run `run_once.sh` to run the simulation for each agent. This takes a while, and can't really be run in the background since the simulator will repeatedly close and reopen. Also, the console will get spammed with red text, but this is fine; when an agent finishes the simulation, I kill ROS to be able to start the next agent as soon as possible.

# Modifying parameters of this project
If you'd like to experiment with the parameters of this project, there are several aspects that could be easily modified to see how they affect the results. Feel free to fork this repository and play around with it. Below are some areas that would allow for such level of experimentation. 

 - `ec_driver.py`: Number of generations, agents per generation
 - `ec_agent.py`: Crossover procedure, mutation procedure, aggressiveness of both
 - `functions/reset_for_solo_run.py`: Default KF genome, filename, data directory
 - `sim_ws/src/capstone/src/kf.py`: KF properties and structure
 - `sim_ws/src/swc_localization/src/localization_node.py`: Policy for choosing next destination
 - `sim_ws/src/swc_control/src/control_node.py`: Motion policy, obstacle avoidance (both could definitely use improvement)

