package pathfollowing;

public class RamsetePath extends PurePursuitPath { //I know they should both extend Path but I'm lazy
    /**
     * Desired pose (everything else is same as Pure Pursuit)
     */
    protected Pose2D desiredPose;

    /**
     * Create a new ramsete path with motion profile
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxDeceleration max deceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     * @param maxAngularVelocity max angular velocity of motion profile
     * @param startVelocity starting velocity of motion profile
     * @param endVelocity desired ending of motion profile
     */
    public RamsetePath(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        super(parametric, maxAcceleration, maxDeceleration, maxVelocity, maxAngularVelocity, startVelocity, endVelocity);
    }

    /**
     * Create a new ramsete path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     * @param startVelocity starting velocity of motion profile
     * @param endVelocity desired ending of motion profile
     */
    public RamsetePath(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    /**
     * Create a new ramsete path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration, starting and ending velocities are 0
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     */
    public RamsetePath(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    /**
     * Calculates a {@link DifferentialDriveState} to travel on the ramsete path based on current robot pose
     * @param robotPose current robot {@link Pose2D}
     * @param dt distance passed since last update
     * @param b convergence aggressiveness tuning parameter (b>0, the larger it is the more aggressive it converges to the desired pose)
     * @param Z dampening tuning parameter (0<Z<1, the larger it is the more dampened any changes are)
     * @param adjust_threshold threshold for adaptively regenerating the quintic hermite spline in meters
     * @param newtonsSteps number of steps to run Newton's method for finding closest point on spline
     * @param trackwidth width of drivetrain in meters
     * @return {@link DifferentialDriveState} based on path and current robot pose
     */
    public DifferentialDriveState update(Pose2D robotPose, double dt, double adjust_threshold, int newtonsSteps, double b, double Z, double trackwidth) {
        //get t associated with closest point on spline
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), newtonsSteps, 10);

        //get distance traveled
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 17);

        //get desired pose (closest point on spline)
        desiredPose = parametric.getPose(closestPointT);

        //limit velocity by max velocity and max acceleration
        //v = a * t
        velocity = Math.min(prevVelocity + maxAcceleration * dt, maxVelocity);

        //angular velocity preview distance based on how much distance it takes to completely slow down (worst case)
        double previewDistance = distanceToSlowdown(prevVelocity, 0, maxDeceleration);
        //get preview t value associated with that distance
        double previewT = parametric.getTFromLength(distanceTraveled + previewDistance);
        //find the maximum possible velocity at that t value
        double maxVelocityAtPreview = maxVelocityFromT(previewT);

        //add previewed max velocity and preview distance to array
        previewVelocities.add(new Vector2D(maxVelocityAtPreview, distanceTraveled + previewDistance));
        //remove array values with distance less than traveled distance
        clearOldPreviewed();

        //update max possible current velocity from previews
        velocity = Math.min(getMaxVelocityFromPreviews(), velocity);

        //limit current velocity based on max angular velocity
        if(Double.isFinite(getRadius(closestPointT))) {
            //get max velocity given max angular velocity and curvature
            double maxCurvatureVelocity = maxVelocityFromRadius(getRadius(closestPointT));

            velocity = Math.min(velocity, maxCurvatureVelocity);
        }

        //distance to end
        distanceToEnd = parametric.getLength() - distanceTraveled;

        //max possible current velocity given max deceleration
        maxVelocityToEnd = maxVelocityFromDistance(distanceToEnd, endVelocity, maxDeceleration);
        velocity = Math.min(velocity, maxVelocityToEnd);

        //limit deceleration to max deceleration
        velocity = Math.max(prevVelocity - maxDeceleration * dt, velocity);

        prevVelocity = velocity;

        //adaptively regenerate path if distance from spline is greater than the adjust threshold
        if(parametric.getPoint(closestPointT).distance(robotPose.getPosition()) > adjust_threshold) {
            //current velocity
            Vector2D curVel = new Vector2D(velocity * robotPose.getAngle().cos(), velocity * robotPose.getAngle().sin());
            //current acceleration
            double acc = (velocity - prevVelocity) / dt;
            Vector2D curAcc = new Vector2D(acc * robotPose.getAngle().cos(), acc * robotPose.getAngle().sin());

            //generate new path
            parametric = parametric.getNewPath(robotPose, curVel, curAcc);
            distanceToEnd = parametric.getLength();
        }

        //generate a differential drive state with ramsete
        return RamseteController.ramsete(robotPose, desiredPose, velocity, velocity * getCurvature(closestPointT), b, Z, trackwidth);

    }

    /**
     * Returns the radius of the tangent circle to the path at a t parameter
     * @param t t parameter to get radius of
     * @return the radius of the tangent circle to the path at a t parameter
     */
    public double getRadius(double t) {
        return 1/getCurvature(t);
    }
}
