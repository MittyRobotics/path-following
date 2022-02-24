package com.github.mittyrobotics.pathfollowing;

import java.util.ArrayList;

public class Path {
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
    protected ArrayList<Vector2D> previewVelocities = new ArrayList<>();

    /**
     * Create a new path with motion profile
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxDeceleration max deceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     * @param maxAngularVelocity max angular velocity of motion profile
     * @param startVelocity starting velocity of motion profile
     * @param endVelocity desired ending of motion profile
     */
    public Path(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
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
     * Create a new path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     * @param startVelocity starting velocity of motion profile
     * @param endVelocity desired ending of motion profile
     */
    public Path(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    /**
     * Create a new path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration, starting and ending velocities are 0
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     */
    public Path(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
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

    /**
     * Returns the radius of the tangent circle to the path at a t parameter
     * @param t t parameter to get radius of
     * @return the radius of the tangent circle to the path at a t parameter
     */
    public double getRadius(double t) {
        return 1/getCurvature(t);
    }

}
