package path;

import math.*;
import splines.Parametric;

import java.util.ArrayList;

public class Path {
    private Parametric parametric;

    private double maxAcceleration, maxVelocity, startVelocity, endVelocity, maxDeceleration, maxAngularVelocity;

    public static final double TO_METERS = 0.0254;
    public static final double TO_INCHES = 39.3700787401;

    private double prevVelocity, distanceTraveled, closestPointT, distanceToEnd, maxVelocityToEnd, velocity, purePursuitRadius;
    boolean turnRight;
    Circle tangentCircle = new Circle();
    Point2D lookaheadPoint = new Point2D();
    private ArrayList<Vector2D> previewVelocities = new ArrayList<>();

    public Path(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        this.parametric = parametric;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.endVelocity = endVelocity;

        this.startVelocity = startVelocity;
        this.prevVelocity = startVelocity;
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    public DifferentialDriveState update(Pose2D robotPose, double dt, double lookahead, double end_threshold, double adjust_threshold, int newtonsSteps, double trackwidth) {
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, newtonsSteps, 10);
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 11);

        lookaheadPoint = getLookahead(distanceTraveled, lookahead);

        distanceToEnd = parametric.getLength() - distanceTraveled - (prevVelocity - maxDeceleration * dt) * dt - end_threshold * 4/5;

        maxVelocityToEnd = maxVelocityFromDistance(distanceToEnd, endVelocity, maxDeceleration);

        velocity = Math.min(Math.min(prevVelocity + maxAcceleration * dt, maxVelocityToEnd), maxVelocity);

        double previewDistance = distanceToSlowdown(prevVelocity, 0, maxDeceleration);
        double previewT = parametric.getTFromLength(distanceTraveled + previewDistance);
        double maxVelocityAtPreview = maxVelocityFromT(previewT);

        previewVelocities.add(new Vector2D(maxVelocityAtPreview, distanceTraveled + previewDistance));
        clearOldPreviewed();

        double maxVelocityFromPreviews = getMaxVelocityFromPreviews();

        velocity = Math.min(maxVelocityFromPreviews, velocity);

        if(Double.isFinite(purePursuitRadius)) {
            double maxCurvatureVelocity = maxVelocityFromRadius(purePursuitRadius);

            velocity = Math.min(velocity, maxCurvatureVelocity);
        }

        velocity = Math.max(prevVelocity - maxDeceleration * dt, velocity);

        prevVelocity = velocity;


        tangentCircle.updateFromPoseAndPoint(robotPose, lookaheadPoint);
        purePursuitRadius = tangentCircle.getRadius();
        turnRight = (tangentCircle.orientationOfPoseAndPoint(robotPose, lookaheadPoint) == 1);


        if(parametric.getPoint(closestPointT).distance(robotPose.getPosition()) > adjust_threshold) {
            Vector2D curVel = new Vector2D(velocity * robotPose.getAngle().cos(), velocity * robotPose.getAngle().sin());
            double acc = (velocity - prevVelocity) / dt;
            Vector2D curAcc = new Vector2D(acc * robotPose.getAngle().cos(), acc * robotPose.getAngle().sin());

            parametric = parametric.getNewPath(robotPose, curVel, curAcc);

        }

        return PurePursuitController.purePursuit(purePursuitRadius, velocity, turnRight, trackwidth);
    }

    public void clearOldPreviewed() {
        previewVelocities.removeIf(previewVelocity -> previewVelocity.getY() <= distanceTraveled);
    }

    public DifferentialDriveState update(Pose2D robotPose, double dt, double lookahead, double end_threshold, double trackwidth) {
        return update(robotPose, dt, lookahead, end_threshold, 5*Path.TO_METERS, 10, trackwidth);
    }

    public double getMaxVelocityFromPreviews() {
        double min = Double.POSITIVE_INFINITY;

        for(Vector2D vel : previewVelocities) {
            min = Math.min(min, maxVelocityFromDistance(vel.y-distanceTraveled, vel.x, maxDeceleration));
        }

        return min;
    }

    public double maxVelocityFromT(double t) {
        return maxVelocityFromRadius(1/(getCurvature(t)));
    }

    public double distanceToSlowdown(double curVelocity, double endVelocity, double maxDeceleration) {
        return (curVelocity * curVelocity - endVelocity * endVelocity) / 2 * maxDeceleration;
    }

    public double distanceFromSpline(Parametric parametric, Pose2D robotPose, int newtonsSteps) {
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, newtonsSteps, 10);

        return parametric.getPoint(closestPointT).distance(robotPose.getPosition());
    }

    public double getCurvature(double t) {
        return parametric.getCurvature(t);
    }

    public double getCurvature() {
        return parametric.getCurvature(closestPointT);
    }

    public double maxVelocityFromDistance(double distance, double endVelocity, double maxDeceleration) {
        //vf^2 = vi^2 + 2ad, solve for vi (deceleration = -a)
        if(distance > 0) return Math.sqrt(endVelocity * endVelocity + 2 * maxDeceleration * distance);
        else return 0;
    }

    public double maxVelocityFromRadius(double radius) {
        if(Double.isInfinite(maxAngularVelocity)) return Double.POSITIVE_INFINITY;
        else return Math.abs(radius * maxAngularVelocity);
    }

    public double getAngularVelocityAtPoint(double t, double linearVelocity) {
        return linearVelocity * getCurvature(t);
    }

    public boolean isFinished(Pose2D robotPosition, double threshold) {
        return robotPosition.getPosition().distance(parametric.getPoint(1.0)) <= threshold;
    }

    public Point2D getLookahead(double distanceTraveled, double lookahead) {
        if(distanceTraveled + lookahead > parametric.getLength()) {
            Angle angle = parametric.getAngle(1);
            Point2D endpoint = parametric.getPoint(1);
            double distanceLeft = distanceTraveled + lookahead - parametric.getLength();
            return new Point2D(endpoint.getX() + distanceLeft * angle.cos(), endpoint.getY() + distanceLeft * angle.sin());
        } else {
            return parametric.getPoint(parametric.getTFromLength(distanceTraveled + lookahead));
        }
    }

    public Point2D getLookaheadFromRobotPose(Pose2D robotPose, double lookahead, int newtonsSteps) {
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, newtonsSteps, 10);
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 11);

        return getLookahead(distanceTraveled, lookahead);
    }

    public Parametric getParametric() { return parametric; }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getStartVelocity() {
        return startVelocity;
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

}
