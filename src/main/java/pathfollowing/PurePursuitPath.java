package pathfollowing;

import java.util.ArrayList;

public class PurePursuitPath {
    /**
     * Associated parametric and motion profile parameters
     */
    protected Parametric parametric;
    protected double maxAcceleration, maxVelocity, startVelocity, endVelocity, maxDeceleration, maxAngularVelocity;

    /**
     * Conversion constants
     */
    public static final double TO_METERS = 0.0254;
    public static final double TO_INCHES = 39.3700787401;

    /**
     * Various helper variables
     */
    protected double prevVelocity, distanceTraveled, closestPointT, distanceToEnd, maxVelocityToEnd, velocity, purePursuitRadius;
    protected boolean turnRight;
    protected Circle tangentCircle = new Circle();
    protected Point2D lookaheadPoint = new Point2D();
    protected ArrayList<Vector2D> previewVelocities = new ArrayList<>();

    /**
     * Create a new pure pursuit path with motion profile
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxDeceleration max deceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     * @param maxAngularVelocity max angular velocity of motion profile
     * @param startVelocity starting velocity of motion profile
     * @param endVelocity desired ending of motion profile
     */
    public PurePursuitPath(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        this.parametric = parametric;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.endVelocity = endVelocity;

        this.startVelocity = startVelocity;
        this.prevVelocity = startVelocity;

        distanceToEnd = parametric.getLength();
    }

    /**
     * Create a new pure pursuit path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     * @param startVelocity starting velocity of motion profile
     * @param endVelocity desired ending of motion profile
     */
    public PurePursuitPath(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    /**
     * Create a new pure pursuit path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration, starting and ending velocities are 0
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     */
    public PurePursuitPath(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    /**
     * Calculates a {@link DifferentialDriveState} to continue on the pure pursuit path based on current robot pose
     * @param robotPose current robot {@link Pose2D}
     * @param dt distance passed since last update
     * @param lookahead pure pursuit lookahead in meters
     * @param adjust_threshold threshold for adaptively regenerating the quintic hermite spline in meters
     * @param newtonsSteps number of steps to run Newton's method for finding closest point on spline
     * @param trackwidth width of drivetrain in meters
     * @return {@link DifferentialDriveState} based on path and current robot pose
     */
    public DifferentialDriveState update(Pose2D robotPose, double dt, double lookahead, double adjust_threshold, int newtonsSteps, double trackwidth) {
        //get t associated with closest point on spline
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), newtonsSteps, 5);

        //get distance traveled
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 17);

        //get the lookahead point
        lookaheadPoint = getLookahead(distanceTraveled, lookahead);

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
        if(Double.isFinite(purePursuitRadius)) {
            //get max velocity given max angular velocity and curvature
            double maxCurvatureVelocity = maxVelocityFromRadius(purePursuitRadius);

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

        //find circle tangent to current pose and lookahead
        tangentCircle.updateFromPoseAndPoint(robotPose, lookaheadPoint);
        purePursuitRadius = tangentCircle.getRadius();
        //find if lookahead is turning right or left
        turnRight = (tangentCircle.orientationOfPoseAndPoint(robotPose, lookaheadPoint) == 1);


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

        //generate a differential drive state with pure pursuit
        return PurePursuitController.purePursuit(purePursuitRadius, velocity, turnRight, trackwidth);
    }

    /**
     * Clear previewed velocities that have passed
     */
    public void clearOldPreviewed() {
        previewVelocities.removeIf(previewVelocity -> previewVelocity.getY() <= distanceTraveled);
    }

    /**
     * Returns the maximum possible velocity from velocity previews
     * @return the maximum possible velocity from velocity previews
     */
    public double getMaxVelocityFromPreviews() {
        double min = Double.POSITIVE_INFINITY;

        for(Vector2D vel : previewVelocities) {
            //get max possible current velocity given future velocity and distance from future velocity
            min = Math.min(min, maxVelocityFromDistance(vel.y-distanceTraveled, vel.x, maxDeceleration));
        }

        return min;
    }

    /**
     * Returns maximum current velocity given curvature
     * @param t current t parameter
     * @return maximum current velocity given curvature
     */
    public double maxVelocityFromT(double t) {
        return maxVelocityFromRadius(1/(getCurvature(t)));
    }

    /**
     * Return the distance needed to get from current velocity to end velocity
     * @param curVelocity current velocity
     * @param endVelocity desired end velocity
     * @param maxDeceleration maximum deceleration of the motion profile
     * @return the distance needed to get from current velocity to end velocity
     */
    public double distanceToSlowdown(double curVelocity, double endVelocity, double maxDeceleration) {
        return (curVelocity * curVelocity - endVelocity * endVelocity) / 2 * maxDeceleration;
    }

    /**
     * Return the distance of a {@link Pose2D} from the spline
     * @param parametric parametric to get distance from
     * @param robotPose {@link Pose2D} to get distance of
     * @param newtonsSteps number of steps to run Newton's method
     * @return the distance of a {@link Pose2D} from the spline
     */
    public double distanceFromSpline(Parametric parametric, Pose2D robotPose, int newtonsSteps) {
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), newtonsSteps, 5);

        return parametric.getPoint(closestPointT).distance(robotPose.getPosition());
    }

    /**
     * Returns the curvature of the parametric at t
     * @param t t parameter to get curvature at
     * @return the curvature of the parametric at t
     */
    public double getCurvature(double t) {
        return parametric.getCurvature(t);
    }

    /**
     * Returns the curvature of the parametric at the closest point
     * @return the curvature of the parametric at the closest point
     */
    public double getCurvature() {
        return parametric.getCurvature(closestPointT);
    }

    /**
     * Returns the maximum possible velocity based on a distance from a desired velocity
     * @param distance distance from point of desired velocity
     * @param endVelocity desired velocity at point of calculation
     * @param maxDeceleration maximum deceleration of the motion profile
     * @return the maximum possible velocity based on a distance from a desired velocity
     */
    public double maxVelocityFromDistance(double distance, double endVelocity, double maxDeceleration) {
        //vf^2 = vi^2 + 2ad, find vi (deceleration = -a)
        if(distance > 0) return Math.sqrt(endVelocity * endVelocity + 2 * maxDeceleration * distance);
        else return 0;
    }

    /**
     * Returns maximum possible linear velocity given the radius of circle at a point
     * @param radius the radius of the circle
     * @return maximum possible linear velocity given the radius of circle at a point
     */
    public double maxVelocityFromRadius(double radius) {
        if(Double.isInfinite(maxAngularVelocity)) return Double.POSITIVE_INFINITY;
        //l = w*r
        else return Math.abs(radius * maxAngularVelocity);
    }

    /**
     * Returns the angular velocity at a point given the linear velocity
     * @param t t parameter of the point
     * @param linearVelocity the linear velocity
     * @return the angular velocity at a point given the linear velocity
     */
    public double getAngularVelocityAtPoint(double t, double linearVelocity) {
        return linearVelocity * getCurvature(t);
    }

    /**
     * Returns whether the path is finished based on the end threshold
     * @param robotPosition current robot {@link Pose2D}
     * @param threshold end threshold in meters
     * @return whether the path is finished based on the end threshold
     */
    public boolean isFinished(Pose2D robotPosition, double threshold) {
        return (robotPosition.getPosition().distance(parametric.getPoint(1.0)) <= threshold) || distanceToEnd <= 0;
    }

    /**
     * Returns the lookahead {@link Point2D} based on current distance traveled
     * @param distanceTraveled distance traveled on the spline
     * @param lookahead lookahead distance in meters
     * @return the lookahead {@link Point2D} based on current distance traveled
     */
    public Point2D getLookahead(double distanceTraveled, double lookahead) {
        if(distanceTraveled + lookahead > parametric.getLength()) {
            //if distance traveled is greater than the spline, return the corresponding point
            //along the straight line continuation from the last point on the spline
            Angle angle = parametric.getAngle(1);
            Point2D endpoint = parametric.getPoint(1);
            double distanceLeft = distanceTraveled + lookahead - parametric.getLength();
            return new Point2D(endpoint.getX() + distanceLeft * angle.cos(), endpoint.getY() + distanceLeft * angle.sin());
        } else {
            return parametric.getPoint(parametric.getTFromLength(distanceTraveled + lookahead));
        }
    }

    /**
     * Return the lookahead {@link Point2D} based on robot {@link Pose2D}
     * @param robotPose current robot {@link Pose2D}
     * @param lookahead lookahead distance in meters
     * @param newtonsSteps number of steps of Newton's method
     * @return the lookahead {@link Point2D} based on robot {@link Pose2D}
     */
    public Point2D getLookaheadFromRobotPose(Pose2D robotPose, double lookahead, int newtonsSteps) {
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), newtonsSteps, 5);
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 17);

        return getLookahead(distanceTraveled, lookahead);
    }

    /**
     * Returns the {@link Parametric} associated with the path
     * @return the {@link Parametric} associated with the path
     */
    public Parametric getParametric() { return parametric; }

    /**
     * Returns the maximum acceleration of the motion profile
     * @return the maximum acceleration of the motion profile
     */
    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * Returns the maximum velocity of the motion profile
     * @return the maximum velocity of the motion profile
     */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * Returns the start velocity of the motion profile
     * @return the start velocity of the motion profile
     */
    public double getStartVelocity() {
        return startVelocity;
    }

    /**
     * Returns the end velocity of the motion profile
     * @return the end velocity of the motion profile
     */
    public double getEndVelocity() {
        return endVelocity;
    }

    /**
     * Returns the maximum deceleration of the motion profile
     * @return the maximum deceleration of the motion profile
     */
    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    /**
     * Returns the maximum angular velocity of the motion profile
     * @return the maximum angular velocity of the motion profile
     */
    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

}
