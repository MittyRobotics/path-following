package com.github.mittyrobotics.pathfollowing;

public class DifferentialDriveState {
    /**
     * Variables to store current velocity state
     */
    private double linearVelocity, angularVelocity, leftVelocity, rightVelocity, radius;

    /**
     * Create a {@link DifferentialDriveState} with all 0 values
     */
    public DifferentialDriveState() {}

    /**
     * Updates the {@link DifferentialDriveState} based on a linear velocity, angular velocity, and track width
     * @param linearVelocity linear velocity in meters/second
     * @param angularVelocity angular velocity in in meters/second
     * @param trackWidth width of drivetrain in meters
     */
    public void updateFromLinearAndAngularVelocity(double linearVelocity, double angularVelocity, double trackWidth) {
        //left vel = angular vel * (radius - trackwidth / 2)
        //right vel = angular vel * (radius + trackwidth / 2)
        //radius = linear vel / w
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;

        if(Math.abs(angularVelocity) < 2e-9) {
            this.radius = Double.POSITIVE_INFINITY;
            this.leftVelocity = linearVelocity;
            this.rightVelocity = linearVelocity;
        } else {
            this.radius = linearVelocity / angularVelocity;
            this.leftVelocity = angularVelocity * (radius - trackWidth / 2.);
            this.rightVelocity = angularVelocity * (radius + trackWidth / 2.);
        }
    }

    /**
     * Updates the {@link DifferentialDriveState} based on a linear velocity, radius, and track width
     * @param linearVelocity linear velocity in meters/second
     * @param radius radius of differential circle in meters
     * @param turnRight true if circle is turning right, false otherwise
     * @param trackWidth width of drivetrain in meters
     */
    public void updateFromLinearVelocityAndRadius(double linearVelocity, double radius, boolean turnRight, double trackWidth) {
        if(Double.isInfinite(radius)) {
            this.angularVelocity = 0;
        } else {
            this.angularVelocity = linearVelocity / radius;
        }
        if(turnRight) angularVelocity *= -1;
        updateFromLinearAndAngularVelocity(linearVelocity, this.angularVelocity, trackWidth);
    }

    /**
     * Return the left side velocity
     * @return left side velocity in meters/second
     */
    public double getLeftVelocity() {
        return leftVelocity;
    }

    /**
     * Return the right side velocity
     * @return right side velocity in meters/second
     */
    public double getRightVelocity() {
        return rightVelocity;
    }

    /**
     * Return the linear velocity
     * @return the linear velocity in meters/second
     */
    public double getLinearVelocity() {
        return linearVelocity;
    }

    /**
     * Return the angular velocity
     * @return the angular velocity in meters/second
     */
    public double getAngularVelocity() {
        return angularVelocity;
    }

    /**
     * Returns string representation of left and right velocities in inches/second
     * @return string representation of left and right velocities in inches/second
     */
    public String toString() {
        return "left velocity = " + leftVelocity * PurePursuitPath.TO_INCHES + ", right velocity = " + rightVelocity * PurePursuitPath.TO_INCHES;
    }

    /**
     * Returns string representation of left and right velocities in meters/second
     * @return string representation of left and right velocities in meters/second
     */
    public String toStringMetric() {
        return "left velocity = " + leftVelocity + ", right velocity = " + rightVelocity;
    }
}
