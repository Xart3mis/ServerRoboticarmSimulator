# This file is a sample POST PROCESSOR script to generate programs
# for K1 Osiris with RoboDK

from robodk import *

# ----------------------------------------------------


def pose_2_str(pose):
    """Prints a pose target"""
    [x, y, z, r, p, w] = pose_2_xyzrpw(pose)
    return "X%.3f Y%.3f Z%.3f R%.3f P%.3f W%.3f" % (x, y, z, r, p, w)


def joints_2_str(joints):
    """Prints a joint target"""
    str = ""
    data = ["A1 ", " A2 ", " A3 ", " A4 ", " A5 ", " ", "F", "H", "I", "J", "K", "L"]
    for i in range(len(joints) - 1):
        str = str + ("%s%i" % (data[i], joints[i]))
    str = str[:]
    return str


# ----------------------------------------------------
# Object class that handles the robot instructions/syntax


class RobotPost(object):
    """Robot post object"""

    PROG_EXT = "txt"  # set the program extension

    # other variables
    ROBOT_POST = ""
    ROBOT_NAME = ""
    PROG_FILES = []

    PROG = ""
    LOG = ""
    nAxes = 5

    def __init__(self, robotpost=None, robotname=None, robot_axes=5, **kwargs):
        self.ROBOT_POST = robotpost
        self.ROBOT_NAME = robotname
        self.PROG = ""
        self.LOG = ""
        self.nAxes = robot_axes

    def ProgStart(self, progname):
        self.addline("M31 (Enable Motors)")
        self.addline("M106 (Homing)")

    def ProgFinish(self, progname):
        self.addline("G91 A1 0 A2 -30 A3 40 A4 0 A5 0")
        self.addlineEnd("M32(Disable Motors)")

    def ProgSave(self, folder, progname, ask_user=False, show_result=False):
        progname = progname + "." + self.PROG_EXT
        if ask_user or not DirExists(folder):
            filesave = getSaveFile(folder, progname, "Save program as...")
            if filesave is not None:
                filesave = filesave.name
            else:
                return
        else:
            filesave = folder + "/" + progname
        fid = open(filesave, "w")
        fid.write(self.PROG)
        fid.close()
        print("SAVED: %s\n" % filesave)
        # ---------------------- show result
        if show_result:
            if type(show_result) is str:
                # Open file with provided application
                import subprocess

                p = subprocess.Popen([show_result, filesave])
            else:
                # open file with default application
                import os

                os.startfile(filesave)

            if len(self.LOG) > 0:
                mbox("Program generation LOG:\n\n" + self.LOG)

    def ProgSendRobot(self, robot_ip, remote_path, ftp_user, ftp_pass):
        """Send a program to the robot using the provided parameters. This method is executed right after ProgSave if we selected the option "Send Program to Robot".
        The connection parameters must be provided in the robot connection menu of RoboDK"""
        UploadFTP(self.PROG_FILES, robot_ip, remote_path, ftp_user, ftp_pass)

    def MoveJ(self, pose, joints, conf_RLF=None):
        """Add a joint movement"""
        self.addline("G91 " + joints_2_str(joints))

    def MoveL(self, pose, joints, conf_RLF=None):
        """Add a linear movement"""
        self.addline("G91 " + joints_2_str(joints))

    def MoveC(self, pose1, joints1, pose2, joints2, conf_RLF_1=None, conf_RLF_2=None):
        """Add a circular movement"""
        # self.addline('MOVC ' + pose_2_str(pose1) + ' ' + pose_2_str(pose2))
        self.addline("G91 " + joints_2_str(joints1))
        self.addline("G91 " + joints_2_str(joints2))

    def setFrame(self, pose, frame_id=None, frame_name=None):
        """Change the robot reference frame"""
        # self.addline('BASE_FRAME ' + pose_2_str(pose))

    def setTool(self, pose, tool_id=None, tool_name=None):
        """Change the robot TCP"""
        # self.addline('TOOL_FRAME ' + pose_2_str(pose))

        # Pause in mms

    def Pause(self, time_ms):
        """Pause the robot program"""
        if time_ms < 0:
            self.addline("PAUSE")
        else:
            self.addline("M30T%s" % (time_ms))

    def setSpeed(self, speed_mms):
        """Changes the robot speed (in mm/s)"""
        self.addlog("setSpeed not defined (%.2f mms)" % speed_mms)

    def setAcceleration(self, accel_mmss):
        """Changes the robot acceleration (in mm/s2)"""
        self.addlog("setAcceleration not defined")

    def setSpeedJoints(self, speed_degs):
        """Changes the robot joint speed (in deg/s)"""
        self.SPEED_RADS = speed_degs * pi / 180
        self.addline("M105 %.3f" % self.SPEED_RADS)

    def setAccelerationJoints(self, accel_degss):
        """Changes the robot joint acceleration (in deg/s2)"""
        self.ACCEL_RADSS = accel_degss * pi / 180
        self.addline("M110 %i" % accel_degss)

    def setZoneData(self, zone_mm):
        """Changes the rounding radius (aka CNT, APO or zone data) to make the movement smoother"""
        self.addlog("setZoneData not defined (%.1f mm)" % zone_mm)

    def setDO(self, io_var, io_value):
        """Sets a variable (digital output) to a given value"""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = "OUT[%s]" % str(io_var)
        if type(io_value) != str:  # set default variable value if io_value is a number
            if io_value > 0:
                io_value = "TRUE"
            else:
                io_value = "FALSE"

        # at this point, io_var and io_value must be string values
        self.addline("%s=%s" % (io_var, io_value))

    def waitDI(self, io_var, io_value, timeout_ms=-1):
        """Waits for a variable (digital input) io_var to attain a given value io_value. Optionally, a timeout can be provided."""
        if type(io_var) != str:  # set default variable name if io_var is a number
            io_var = "IN[%s]" % str(io_var)
        if type(io_value) != str:  # set default variable value if io_value is a number
            if io_value > 0:
                io_value = "TRUE"
            else:
                io_value = "FALSE"

        # at this point, io_var and io_value must be string values
        if timeout_ms < 0:
            self.addline("WAIT FOR %s==%s" % (io_var, io_value))
        else:
            self.addline(
                "WAIT FOR %s==%s TIMEOUT=%.1f" % (io_var, io_value, timeout_ms)
            )

    def RunCode(self, code, is_function_call=False):
        """Adds code or a function call"""
        if is_function_call:
            code.replace(" ", "_")
            if not code.endswith(")"):
                code = code + "()"
            self.addline(code)
        else:
            self.addline(code)

    def RunMessage(self, message, iscomment=False):
        """Display a message in the robot controller screen (teach pendant)"""
        if iscomment:
            # self.addline('% ' + message)
            if message == "Attach to Gripper V2":
                self.addline("M21")
            elif message == "Detach from Gripper V2":
                self.addline("M20")
        else:
            self.addlog("Show message on teach pendant not implemented (%s)" % message)

    # ------------------ private ----------------------
    def addlineEnd(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + newline

    def addline(self, newline):
        """Add a program line"""
        self.PROG = self.PROG + newline + "\n"

    def addlog(self, newline):
        """Add a log message"""
        self.LOG = self.LOG + newline + "\n"


# -------------------------------------------------
