import pybullet as p
import pybullet_data
import time

# Connect to simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load environment
p.loadURDF("plane.urdf")
table = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, 0])
robot = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0], useFixedBase=True)
block = p.loadURDF("cube_small.urdf", basePosition=[0.7, 0, 0.65])

# Let everything settle
for _ in range(100):
    p.stepSimulation()
    time.sleep(1/240)

# Get robot end effector link index
num_joints = p.getNumJoints(robot)
end_effector_index = num_joints - 1  # Last joint

# Define target position above the cube
target_position = [0.7, 0, 0.75]
target_orientation = p.getQuaternionFromEuler([0, 3.14, 0])  # Flip wrist downward

# Use inverse kinematics to get joint angles
joint_poses = p.calculateInverseKinematics(robot, end_effector_index, target_position, target_orientation)

# Apply joint positions to move arm
for i in range(len(joint_poses)):
    p.setJointMotorControl2(bodyIndex=robot,
                            jointIndex=i,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_poses[i])

# Let the arm settle into position before grasping
for _ in range(100):
    p.stepSimulation()
    time.sleep(1/240)

# Attach the cube to the robot's end-effector (fake grasp)
cid = p.createConstraint(
    parentBodyUniqueId=robot,
    parentLinkIndex=end_effector_index,
    childBodyUniqueId=block,
    childLinkIndex=-1,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0]
)

# Keep simulation running
while True:
    p.stepSimulation()
    time.sleep(1/240)
