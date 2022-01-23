package pathfollowing;

public class RamseteController {

    /**
     * Variables for ramsete calculations (for debugging)
     */
    public static double ex, ey, et, k, rvel, rAngVel, vel, angVel;
    public static Angle t, td;

    /**
     * Returns a {@link DifferentialDriveState} based on a current {@link Pose2D}, desired {@link Pose2D}, desired linear velocity, desired angular velocity, tuning constants
     * @param curPose current {@link Pose2D}
     * @param desiredPose desired {@link Pose2D}
     * @param desiredVelocity desired linear velocity in meters/second
     * @param desiredAngularVelocity desired angular velocity in meters/second
     * @param b convergence aggressiveness tuning parameter (b>0, the larger it is the more aggressive it converges to the desired pose)
     * @param Z dampening tuning parameter (0<Z<1, the larger it is the more dampened any changes are)
     * @param trackwidth width of the drivetrain in meters
     * @return a {@link DifferentialDriveState} based on a current {@link Pose2D}, desired {@link Pose2D}, desired linear velocity, desired angular velocity, tuning constants
     */
    public static DifferentialDriveState ramsete(Pose2D curPose, Pose2D desiredPose, double desiredVelocity, double desiredAngularVelocity, double b, double Z, double trackwidth) {
        //https://file.tavsys.net/control/controls-engineering-in-frc.pdf (literally copied from this)

        //b > 0, 0 < Z < 1, larger b -> faster convergence, larger Z -> more dampening

        vel = desiredVelocity;
        angVel = desiredAngularVelocity;

        k = 2*Z*Math.sqrt(desiredAngularVelocity*desiredAngularVelocity + b * desiredVelocity * desiredVelocity);

        t = curPose.getAngle();
        td = desiredPose.getAngle();
        double x = curPose.getPosition().getX();
        double xd = desiredPose.getPosition().getX();
        double y = curPose.getPosition().getY();
        double yd = desiredPose.getPosition().getY();

        ex = t.cos() * (xd - x) + t.sin() * (yd - y);
        ey = t.sin() * (x - xd) + t.cos() * (yd - y);
        et = td.getAngleBetween(t);

        //get linear and angular velocities as calculated by ramsete
        rvel = desiredVelocity * Math.cos(et) + k * ex;
        rAngVel = desiredAngularVelocity + k * et + b * desiredVelocity * sinc(et) * ey;

        DifferentialDriveState dds = new DifferentialDriveState();

        dds.updateFromLinearAndAngularVelocity(rvel, rAngVel, trackwidth);
        return dds;
    }

    //sinc function (sin(x)/x)
    public static double sinc(double e) {
        //limit of sin(x)/x as x->0 is 1, L'Hospital's Rule if you've taken calculus
        if(e == 0) return 1;
        return Math.sin(e)/e;
    }
}
