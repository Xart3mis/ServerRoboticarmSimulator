from websocket_server import WebsocketServer

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


def new_client(client, server):
    print("New client connected and was given id %d" % client["id"])
    server.send_message_to_all("Hey all, a new client has joined us")


def client_disconnect(client, server):
    print("Client(%d) disconnected" % client["id"])


def message_received(client, server, message):
    if not prog.Busy() and not robot.Busy():
        move_direction = [0, 0, 0]

        if message == "Y-":
            move_direction = [0, -1, 0]
        elif message == "Y+":
            move_direction = [0, 1, 0]
        elif message == "X-":
            move_direction = [-1, 0, 0]
        elif message == "X+":
            move_direction = [1, 0, 0]
        elif message == "Z+":
            move_direction = [0, 0, 1]
        elif message == "Z-":
            move_direction = [0, 0, -1]
        elif message == "home":
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
        print(message)

        return ":D"

    print("Client(%d) said: %s" % (client["id"], message))

    return "\nmashy\n"


server = WebsocketServer(host="0.0.0.0", port=6969)
server.set_fn_new_client(new_client)
server.set_fn_client_left(client_disconnect)
server.set_fn_message_received(message_received)
server.run_forever()
