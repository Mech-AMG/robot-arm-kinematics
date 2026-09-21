import math


def forward_kinematics(length1, length2, angle1, angle2):
    """Return the end-effector (x, y) for a planar two-link arm."""
    if length1 <= 0 or length2 <= 0:
        raise ValueError("Link lengths must be positive.")

    first = math.radians(angle1)
    second = math.radians(angle1 + angle2)
    x = length1 * math.cos(first) + length2 * math.cos(second)
    y = length1 * math.sin(first) + length2 * math.sin(second)
    return x, y


def inverse_kinematics(length1, length2, x, y):
    """Return elbow-up and elbow-down joint solutions for (x, y)."""
    if length1 <= 0 or length2 <= 0:
        raise ValueError("Link lengths must be positive.")

    distance_squared = x**2 + y**2
    minimum = abs(length1 - length2)
    maximum = length1 + length2
    distance = math.sqrt(distance_squared)
    if distance < minimum - 1e-9 or distance > maximum + 1e-9:
        raise ValueError("Target is outside the arm's reachable workspace.")

    cosine = (distance_squared - length1**2 - length2**2) / (2 * length1 * length2)
    cosine = max(-1.0, min(1.0, cosine))
    elbow_angle = math.degrees(math.acos(cosine))
    base_angle = math.degrees(math.atan2(y, x))
    shoulder_offset = math.degrees(
        math.atan2(length2 * math.sin(math.radians(elbow_angle)),
                  length1 + length2 * math.cos(math.radians(elbow_angle)))
    )
    elbow_up = (base_angle - shoulder_offset, elbow_angle)
    elbow_down = (
        base_angle + shoulder_offset,
        -elbow_angle,
    )
    return elbow_up, elbow_down


def forward_kinematics_3d(length1, length2, yaw, shoulder, elbow):
    """Return (x, y, z) for a spherical-base two-link arm."""
    if length1 <= 0 or length2 <= 0:
        raise ValueError("Link lengths must be positive.")

    yaw_radians = math.radians(yaw)
    shoulder_radians = math.radians(shoulder)
    elbow_radians = math.radians(shoulder + elbow)
    radial = (
        length1 * math.cos(shoulder_radians)
        + length2 * math.cos(elbow_radians)
    )
    z = (
        length1 * math.sin(shoulder_radians)
        + length2 * math.sin(elbow_radians)
    )
    return radial * math.cos(yaw_radians), radial * math.sin(yaw_radians), z


def inverse_kinematics_3d(length1, length2, x, y, z):
    """Return elbow-up and elbow-down (yaw, shoulder, elbow) solutions."""
    radial = math.hypot(x, y)
    planar_solutions = inverse_kinematics(length1, length2, radial, z)
    yaw = math.degrees(math.atan2(y, x))
    return tuple(
        (yaw, shoulder, elbow) for shoulder, elbow in planar_solutions
    )


def interpolate_joints(start, end, steps):
    """Return evenly spaced joint-angle points, including both endpoints."""
    if steps < 2:
        raise ValueError("Steps must be at least 2.")
    if len(start) != len(end) or not start:
        raise ValueError("Joint positions must have matching angles.")

    return [
        tuple(
            start[axis] + (end[axis] - start[axis]) * index / (steps - 1)
            for axis in range(len(start))
        )
        for index in range(steps)
    ]


def read_float(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a number.")


def print_solution(solution):
    labels = ("Joint 1", "Joint 2") if len(solution) == 2 else (
        "Base yaw", "Shoulder", "Elbow"
    )
    for label, angle in zip(labels, solution):
        print(f"  {label}: {angle:.2f} degrees")


def main():
    print("Two-Link Robot Arm Kinematics")
    length1 = read_float("Length of link 1: ")
    length2 = read_float("Length of link 2: ")
    mode = input("Mode (2D or 3D): ").strip().upper()
    if mode not in {"2D", "3D"}:
        print("Mode must be 2D or 3D.")
        return

    while True:
        print("\nChoose an operation:")
        print("1. Forward kinematics")
        print("2. Inverse kinematics")
        print("3. Generate joint trajectory")
        print("4. Exit")
        choice = input("Selection: ").strip()

        try:
            if choice == "1":
                if mode == "2D":
                    angle1 = read_float("Shoulder angle (degrees): ")
                    angle2 = read_float("Elbow angle (degrees): ")
                    position = forward_kinematics(
                        length1, length2, angle1, angle2
                    )
                else:
                    yaw = read_float("Base yaw angle (degrees): ")
                    shoulder = read_float("Shoulder elevation (degrees): ")
                    elbow = read_float("Elbow angle (degrees): ")
                    position = forward_kinematics_3d(
                        length1, length2, yaw, shoulder, elbow
                    )
                print("End effector: " + ", ".join(
                    f"{axis}={value:.2f}"
                    for axis, value in zip(("x", "y", "z"), position)
                ))
            elif choice == "2":
                x = read_float("Target x: ")
                y = read_float("Target y: ")
                if mode == "2D":
                    solutions = inverse_kinematics(length1, length2, x, y)
                else:
                    z = read_float("Target z: ")
                    solutions = inverse_kinematics_3d(
                        length1, length2, x, y, z
                    )
                print("Elbow-up solution:")
                print_solution(solutions[0])
                print("Elbow-down solution:")
                print_solution(solutions[1])
            elif choice == "3":
                dimensions = 2 if mode == "2D" else 3
                labels = (
                    ("shoulder", "elbow")
                    if dimensions == 2
                    else ("yaw", "shoulder", "elbow")
                )
                start = tuple(
                    read_float(f"Start {label} angle: ") for label in labels
                )
                end = tuple(
                    read_float(f"End {label} angle: ") for label in labels
                )
                steps = int(read_float("Number of trajectory points: "))
                for index, point in enumerate(interpolate_joints(start, end, steps), 1):
                    values = ", ".join(
                        f"{label}={angle:.2f}"
                        for label, angle in zip(labels, point)
                    )
                    print(f"{index:>3}: {values}")
            elif choice == "4":
                print("Goodbye.")
                return
            else:
                print("Choose an option from 1 to 4.")
        except ValueError as error:
            print(f"Error: {error}")


if __name__ == "__main__":
    main()
