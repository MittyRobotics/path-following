package path;

import math.*;
import splines.Parametric;
import splines.QuinticHermiteSpline;

public class Path {
    private Parametric parametric;
    private double maxAcceleration, maxVelocity, endVelocity, maxDeceleration, maxAngularVelocity;

    public static final double TO_METERS = 0.0254;
    public static final double TO_INCHES = 39.3700787401;

    private double prevVelocity, distanceTraveled;

    public Path(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        this.parametric = parametric;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.endVelocity = endVelocity;

        this.prevVelocity = startVelocity;
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    public DifferentialDriveState update(Pose2D robotPose, double dt, double lookahead, double threshold, int newtonsSteps, double trackwidth) {
        double closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, newtonsSteps, 10);
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 11);

        Point2D lookaheadPoint = getLookahead(distanceTraveled, lookahead);

//        parametric.getPoint(closestPointT).print();

        double distanceToEnd = parametric.getLength() - distanceTraveled;
        double maxVelocityToEnd = maxVelocityFromDistance(distanceToEnd, endVelocity, maxDeceleration);


        double velocity = Math.min(Math.min(prevVelocity + maxAcceleration * dt, maxVelocityToEnd), maxVelocity);


        Circle tangentCircle = new Circle();
        tangentCircle.updateFromPoseAndPoint(robotPose, lookaheadPoint);
        double purePursuitRadius = tangentCircle.getRadius();
        boolean turnRight = (tangentCircle.orientationOfPoseAndPoint(robotPose, lookaheadPoint) == 1);

        if(Double.isFinite(purePursuitRadius)) {
            double maxCurvatureVelocity = maxVelocityFromCurvature(purePursuitRadius);
            velocity = Math.min(velocity, maxCurvatureVelocity);
        }

        prevVelocity = velocity;

        if(parametric.getPoint(closestPointT).distance(robotPose.getPosition()) > threshold) {
            System.out.println(parametric.getPoint(closestPointT).distance(robotPose.getPosition()));
            Vector2D curVel = new Vector2D(velocity * robotPose.getAngle().cos(), velocity * robotPose.getAngle().sin());
            double acc = (velocity - prevVelocity) / dt;
            Vector2D curAcc = new Vector2D(acc * robotPose.getAngle().cos(), acc * robotPose.getAngle().sin());

            parametric = parametric.getNewPath(robotPose, curVel, curAcc);

            closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, 10, 10);
            distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 11);

        }

        return PurePursuitController.purePursuit(purePursuitRadius, velocity, turnRight, trackwidth);
    }

    public DifferentialDriveState update(Pose2D robotPose, double dt, double lookahead, double trackwidth) {
        return update(robotPose, dt, lookahead, 5*Path.TO_METERS, 10, trackwidth);
    }

    public double distanceFromSpline(Parametric parametric, Pose2D robotPose) {
        double closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, 10, 10);
        return parametric.getPoint(closestPointT).distance(robotPose.getPosition());
    }

    public double getCurvature(Pose2D robotPose) {
        double closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, 10, 10);
        return parametric.getCurvature(closestPointT);
    }

    public double maxVelocityFromDistance(double distance, double endVelocity, double maxDeceleration) {
        //vf^2 = vi^2 + 2ad, solve for vi (deceleration = -a)
        if(distance > 0) return Math.sqrt(endVelocity * endVelocity + 2 * maxDeceleration * distance);
        else return 0;
    }

    public double maxVelocityFromCurvature(double radius) {
        if(Double.isInfinite(maxAngularVelocity)) return Double.POSITIVE_INFINITY;
        else return Math.abs(radius * maxAngularVelocity);
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

    public Parametric getParametric() { return parametric; }

}
