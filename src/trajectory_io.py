import json

from drive.skid_steer_drive import skid_steer_drive
from obstacle.circular_obstacle import circular_obstacle
# from obstacle.rectangular_obstangle import rectangular_obstacle

def load_json(file):
    return json.load(open(file))
    
def import_robot(solver, file):
    drive = load_json(file)
    type = drive["type"]
    if type == "skid steer":
        return skid_steer_drive(solver, 
            drive["length"], 
            drive["width"], 
            drive["wheelbase"], 
            drive["dynamics"]["kv"], 
            drive["dynamics"]["ka"])
    elif type == "swerve":
        print("swerve")
    else:
        print("This drive type is not supported!")
    return 0

def import_obstacles(file):
    field = load_json(file)
    obstacles = []
    for obstacle in field["obstacles"]:
        type = obstacle["type"]
        if type == "circle":
            obstacles.append(circular_obstacle(
                obstacle["x"],
                obstacle["y"],
                obstacle["radius"]
            ))
        # elif type == "rectangle":
        #     obstacles.append(rectangular_obstacle(
        #         obstacle["x"],
        #         obstacle["y"],
        #         obstacle["theta"],
        #         obstacle["length"],
        #         obstacle["width"]
        #     ))
        else:
            print("This obstacle type is not supported!")
    return obstacles