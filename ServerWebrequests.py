import json
from time import sleep

import flask

# from smbus import SMBus
from flask import request
from flask_limiter import Limiter
from flask_limiter.util import get_remote_address
from getch import getch

from robodk import *
from robolink import *

currentPath = os.path.dirname(os.path.realpath(__file__))

os.chdir(currentPath)

RDK = Robolink(
    args=[
        "-NOSPLASH",
        "-TREE_STATE=0",
        "-EXIT_LAST_COMM",
        "-NOUI",
        "RoboDk/station.rdk",
    ],
)

RDK.Render(False)
RDK.setRunMode(RUNMODE_SIMULATE)
RDK.setCollisionActive(COLLISION_OFF)
RDK.setWindowState(WINDOWSTATE_MAXIMIZED)

prog = RDK.AddProgram("AutoProgram")
prog.ShowInstructions(False)

robot = RDK.Item("K1.1 Osiris", ITEM_TYPE_ROBOT)
# robot.setParam("PostProcessor", "K1 Osiris Post.py")

robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())
Speed = 50


def Home(rbt):
    rbt.MoveJ(robot.JointsHome())


Home(robot)

"""
addr = 8
bus = SMBus(0)

data = []
"""
app = flask.Flask(__name__)
app.config["ENV"] = "development"
app.config["DEBUG"] = True
app.config["TESTING"] = True

limiter = Limiter(app, key_func=get_remote_address)


@app.route("/", methods=["GET"])
def homePage():
    return "<h1>Controller Api</h1>"


@app.errorhandler(404)
def page_not_found(e):
    return "<h1>404</h1><p>The resource could not be found.</p>"


@limiter.limit("1/second")
@app.route("/", methods=["POST"])
def handle():
    if not prog.Busy() and not robot.Busy():
        jsonData = request.get_data()
        jsonData = json.loads(jsonData)

        key = jsonData["direction"]

        move_direction = [0, 0, 0]

        if key == "Y-":
            move_direction = [0, -1, 0]
        elif key == "Y+":
            move_direction = [0, 1, 0]
        elif key == "X-":
            move_direction = [-1, 0, 0]
        elif key == "X+":
            move_direction = [1, 0, 0]
        elif key == "Z+":
            move_direction = [0, 0, 1]
        elif key == "Z-":
            move_direction = [0, 0, -1]
        elif key == "home":
            Home(robot)

        if norm(move_direction) <= 0:
            return ""

        move_direction = mult3(move_direction, Speed)

        Joints = robot.Joints()
        CurrentPosition = robot.SolveFK(Joints)

        NewPosition = CurrentPosition * transl(move_direction)
        NewPosition = robot.SolveIK(NewPosition)

        if len(NewPosition.tolist()) < 6:
            return "no solution"

        robot.MoveJ(NewPosition)

        if not robot.Busy():
            RDK.Update()

        robot.RunInstruction("Program_Done")
        # bus.write_i2c_block_data(addr, 0, data)
        print(jsonData)

        return ":D"

    return "\nmashy\n"


app.run(host="0.0.0.0", port=6969)
