# This file will read results.txt after it is created by the simulator finishing a run.

def read_file():
    # bring in results.txt as a list of strings for each line
    # open file in read mode.
    file1 = open("results.txt", "r+")
    results = file1.readlines()
    file1.close()
    # check for error
    if results[0][0:5] == "Error":
        return {"Score":800}
    
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
    elements["Num waypoints hit"] = 0
    elements["Num waypoints hit"] += 1 if "Waypoint 1 reached" in elements else 0
    elements["Num waypoints hit"] += 1 if "Waypoint 2 reached" in elements else 0
    elements["Num waypoints hit"] += 1 if "Waypoint 3 reached" in elements else 0
    elements["wp1"] = "Waypoint 1 reached" in elements
    elements["wp2"] = "Waypoint 2 reached" in elements
    elements["wp3"] = "Waypoint 3 reached" in elements
    # convert the types of important vars
    elements["Time"] = float(elements["Time"])
    elements["Score"] = float(elements["Score"])
    elements["Multiplier"] = float(elements["Multiplier"][1:]) #remove the "x"
    # recover the sim settings
    elements["Seed"] = int(elements["Seed"])
    obs_settings = {"x1.00":"normal", "x15.00":"none", "x0.10":"hard"}
    noise_settings = {"x10.00":"none", "x1.00":"reduced", "x0.60":"realistic"}
    elements["Obstacle Type"] = obs_settings[elements["Obstacles Bonus"][:-1]]
    elements["Noise Type"] = noise_settings[elements["Noise Bonus"][:-1]]

    return elements
    
h_keys = ["Seed", "Obstacle Type", "Noise Type", "Time", "Score", "Damage", "Num waypoints hit","wp1","wp2","wp3"]
header = [s.lower().replace(" ","_") for s in h_keys]

def write_file(elements):
    #filepath = "results/" + str(elements["Seed"]) + ".csv"
    filepath = "config/results.csv"
    file2 = open(filepath, "w+")
    # clear the file
    file2.seek(0)
    file2.truncate()
    # add the header
    file2.write(",".join(header) + "\n")
    # add the row for this round of results
    row = ",".join([str(elements[k]) for k in h_keys])
    file2.write(row + "\n")
    file2.close()

#el = read_file()
#print(el)
#write_file_full(el)


""" # Sample results.txt on successful completion of course.
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

""" #Sample results.txt on simulator shutdown.
Error: Connection to RosBridge lost!
"""
