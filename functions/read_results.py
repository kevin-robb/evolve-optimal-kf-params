# This file will read results.txt after it is created by the simulator finishing a run.

def read_file():
    # bring in results.txt as a list of strings for each line
    # TODO
    results = []
    # assume it is stored in results, and empty lines are skipped
    
    # interpret the important elements, and write to dictionary.
    keys, values = [], []
    for line in results:
        # lines with important info all contain ": "
        if ":" in line:
            l = line.split(": ")
            # check if this is the "XX% Damaged" key
            if "%" in l[0]:
                keys.append("Damage")
                values.append(float(l[1][1:])) #remove the "x"
            else:
                keys.append(l[0])
                values.append(l[1])
    # make the dictionary
    elements = dict(zip(keys, values))
    # if the robot was not damaged, add the value of 0 so it doesn't break things
    if "Damage" not in keys:
        elements["Damage"] = 0.0
    # count the number of waypoints reached
    elements["Number of waypoints reached"] = 0
    elements["Number of waypoints reached"] += 1 if "Waypoint 1 reached" in elements else 0
    elements["Number of waypoints reached"] += 1 if "Waypoint 2 reached" in elements else 0
    elements["Number of waypoints reached"] += 1 if "Waypoint 3 reached" in elements else 0
    # convert the types of important vars
    elements["Time"] = float(elements["Time"])
    elements["Score"] = float(elements["Score"])
    elements["Multiplier"] = float(elements["Multiplier"][1:]) #remove the "x"
    # recover the sim settings
    elements["Seed"] = int(elements["Seed"])
    obs_settings = {"x1.00":"normal", "x15.00":"none", "x0.10":"hard"}
    noise_settings = {"x10.00":"none", "x1.00":"reduced", "x0.60":"realistic"}
    elements["Obstacle Type"] = obs_settings[elements["Obstacle Bonus"]]
    elements["Noise Type"] = noise_settings[elements["Noise Bonus"]]
    # TODO do something with this stuff


"""
Team WickedSlickRobotics
Date: 2021-02-23 17:50:17 -06
Seed: 76168294

Time: 24.680
Multiplier: x1.539
Score: 37.986

-- Multipliers --
Obstacles Bonus: x1.00
Noise Bonus: x0.60
Waypoint 1 reached: x0.80
Waypoint 2 reached: x0.80
Waypoint 3 reached: x0.80
44% Damaged: x5.01
"""