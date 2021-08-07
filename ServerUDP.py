import socket

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
moveSpeed = 50


def Home(rbt):
    rbt.MoveJ(robot.JointsHome())


Home(robot)

port = 6969
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", port))

print("waiting on port: ", port)

while True:
    data, addr = s.recvfrom(1024)
    print(data)
    if not prog.Busy() and not robot.Busy():
        move_direction = [0, 0, 0]

        if data == "Y-":
            move_direction = [0, -1, 0]
        elif data == "Y+":
            move_direction = [0, 1, 0]
        elif data == "X-":
            move_direction = [-1, 0, 0]
        elif data == "X+":
            move_direction = [1, 0, 0]
        elif data == "Z+":
            move_direction = [0, 0, 1]
        elif data == "Z-":
            move_direction = [0, 0, -1]
        elif data == "home":
            Home(robot)

        if norm(move_direction) <= 0:
            continue

        xyz_move = mult3(move_direction, moveSpeed)

        robot_joints = robot.Joints()

        robot_position = robot.SolveFK(robot_joints)

        robot_config = robot.JointsConfig(robot_joints)

        new_robot_position = transl(xyz_move) * robot_position

        new_robot_joints = robot.SolveIK(new_robot_position)
        if len(new_robot_joints.tolist()) < 6:
            print(
                "No robot solution!! The new position is too far, out of reach or close to a singularity"
            )
            continue

        new_robot_config = robot.JointsConfig(new_robot_joints)

        robot.MoveJ(new_robot_joints)
