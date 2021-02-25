# This file will read results.txt after it is created by the simulator finishing a run.

def read_file():
    # bring in results.txt as a list of strings for each line
    # open file in read mode.
    file1=open("results.txt", "r+")
    results = file1.readlines()
    # assume empty lines are skipped
    
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
    elements["Obstacle Type"] = obs_settings[elements["Obstacles Bonus"][:-1]]
    elements["Noise Type"] = noise_settings[elements["Noise Bonus"][:-1]]
    # TODO do something with this stuff
    print(elements)

read_file()

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

""" from my decision tree, reading from file
def file_to_tree(self, filename:str=None) -> Node:
        # read from a file and store the contents as a tree
        if filename is None:
            filename = self.filename
        # open file in read mode.
        file1=open("tree_storage/" + filename + ".txt", "r+")
        # read each line & return as each a string element in a list.
        file_content = file1.readlines()
        # initialize array to store data from file.
        self.arr = [None for i in range(len(file_content))]
        # go through and parse each line as a row in arr.
            # -2,None,None,None: no node.
            # 1,int,None,None,int: terminal nodes.
            # 1,int,int,float,None: other nodes.
        # make sure to skip None rows (denoted "-" in file).
        for i in range(len(file_content)):
            row_str = file_content[i].split(",")
            if int(row_str[0]) == -2: # no node. leave as None.
                continue
            elif row_str[2] == "None": # terminal node.
                self.arr[i] = [int(row_str[1]),None,None,int(row_str[4])]
            else: # non-terminal node.
                self.arr[i] = [int(row_str[1]),int(row_str[2]),float(row_str[3]), None]
        # convert arr to tree and return
        self.root = self.arr_to_tree()
        return self.root
"""