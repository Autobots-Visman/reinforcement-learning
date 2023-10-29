import sys

import moveit_commander
import rospy
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from rospy_message_converter import message_converter

app = FastAPI()

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface", anonymous=True)

app.mount("/", StaticFiles(directory="../static", html=True), name="static")


@app.get("/state")
async def state():
    robot = moveit_commander.RobotCommander()
    state = robot.get_current_state()
    serialized = message_converter.convert_ros_message_to_dictionary(state)
    return {"robot": serialized}
