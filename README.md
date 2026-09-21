# RoboKinematics

#### Video Demo: [Watch the demo on YouTube](https://youtu.be/GHFFKyi5DwY?si=vJbCZmRVGCNZZWNS)

#### Description:

RoboKinematics is a command-line engineering tool for analyzing two-link robot
arms in both 2D and 3D. It was designed for a mechatronics workflow in which a designer
needs to move between physical dimensions, joint angles, and Cartesian
coordinates without repeatedly doing the trigonometry by hand. The program
supports forward kinematics, inverse kinematics, and simple joint-space
trajectory generation in either planar 2D or spherical-base 3D.

The arm is modeled as two rigid links connected by a shoulder joint and an
elbow joint. In 2D, both links move in the x-y plane. In 3D, an additional
base-yaw joint rotates that planar arm around the z-axis, allowing the
end-effector to reach x, y, and z coordinates. Link lengths are entered in any
consistent unit, such as millimeters or centimeters. Angles are entered in
degrees. Forward kinematics uses the link lengths and joint angles to calculate
the end-effector position.

Inverse kinematics works in the opposite direction. Given a target x and y
position in 2D, or x, y, and z in 3D, the program checks whether the point is
inside the arm's reachable workspace. If it is reachable, it reports both the
elbow-up and elbow-down
solutions. Returning both configurations is important in robotics because the
same point can often be reached with different poses, and one pose may avoid
an obstacle or fit the mechanical design better than the other.

The trajectory option creates evenly spaced joint-angle points between a
starting pose and an ending pose. These points can be used as a simple
reference sequence for a motor controller or as sample data for a later
simulation. The program includes input validation for positive link lengths,
unreachable targets, malformed numeric input, and invalid trajectory sizes.

`project.py` contains the interactive application and the reusable
`forward_kinematics`, `inverse_kinematics`, `forward_kinematics_3d`,
`inverse_kinematics_3d`, and `interpolate_joints` functions.
`test_project.py` tests the mathematical behavior, verifies that inverse
kinematics reconstructs its requested target, and checks important error cases.
`requirements.txt` lists pytest, which is used to run the automated tests.

To run the program, execute `python project.py` from this directory. To run the
tests, install the dependencies with `pip install -r requirements.txt`, then
execute `pytest test_project.py`. The project intentionally uses only Python's
standard `math` module at runtime, keeping it easy to run on a laptop,
embedded-development workstation, or lab computer.
