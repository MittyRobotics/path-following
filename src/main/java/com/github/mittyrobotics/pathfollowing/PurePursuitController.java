package com.github.mittyrobotics.pathfollowing;

public class PurePursuitController {
    /**
     * Returns a {@link DifferentialDriveState} based on the radius of the tangent circle, linear velocity, and track width
     * @param tangentRadius radius of the tangent circle in meters
     * @param linearVelocity linear velocity in meters/second
     * @param turnRight whether the tangent circle is turning right
     * @param trackwidth width of the drivetrain in meters
     * @return a {@link DifferentialDriveState} based on the radius of the tangent circle, linear velocity, and track width
     */
    public static DifferentialDriveState purePursuit(double tangentRadius, double linearVelocity, boolean turnRight, double trackwidth) {
        DifferentialDriveState dds = new DifferentialDriveState();
        dds.updateFromLinearVelocityAndRadius(linearVelocity, tangentRadius, turnRight, trackwidth);
        return dds;
    }
}
