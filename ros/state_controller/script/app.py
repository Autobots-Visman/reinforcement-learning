import moveit_commander
import rospy
from fastapi import FastAPI

app = FastAPI()


@app.get("/")
async def root():
    return {"message": "Hello World"}


@app.get("/state")
async def state():
    robot = moveit_commander.RobotCommander()
    state = robot.get_current_state()
    return {"message": state}
