package com.github.mittyrobotics.pathfollowing;

import java.util.ArrayList;

public class PurePursuitPath extends Path {
    /**
     * Various helper variables
     */
    protected Circle tangentCircle = new Circle();
    protected Point2D lookaheadPoint = new Point2D();

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
        super(parametric, maxAcceleration, maxDeceleration, maxVelocity, maxAngularVelocity, startVelocity, endVelocity);
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
        super(parametric, maxAcceleration, maxVelocity, startVelocity, endVelocity);
    }

    /**
     * Create a new pure pursuit path with motion profile, max angular velocity is infinity, max deceleration is equal to max acceleration, starting and ending velocities are 0
     * @param parametric parametric associated with path
     * @param maxAcceleration max acceleration of motion profile
     * @param maxVelocity max velocity of motion profile
     */
    public PurePursuitPath(Parametric parametric, double maxAcceleration, double maxVelocity) {
        super(parametric, maxAcceleration, maxVelocity);
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

}
