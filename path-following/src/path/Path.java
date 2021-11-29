package path;

import math.Point2D;
import math.Pose2D;
import math.Vector2D;
import splines.Parametric;

public class Path {
    private Parametric parametric;
    private double maxAcceleration, maxVelocity, startVelocity, endVelocity, maxDeceleration, maxAngularVelocity, totalDistance;

    private Pose2D prevRobotPosition;
    private double prevVelocity;

    public Path(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        this.parametric = parametric;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.startVelocity = 0;
        this.endVelocity = 0;

        totalDistance = parametric.getGaussianQuadratureLength(11);
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    public DifferentialDriveState update(Pose2D robotPosition, double dt, double lookahead, double trackwidth) {
        double closestPointT = parametric.findClosestPointOnSpline(robotPosition.getPosition(), 0.01, 10, 10);

        double lookaheadT = closestPointT + lookahead;

        Point2D lookaheadPoint = parametric.getPoint(lookaheadT);

        double distanceToEnd = parametric.getGaussianQuadratureLength(closestPointT, 1.0, 11);

        double maxVelocityToEnd = maxVelocityFromDistance(distanceToEnd, endVelocity, maxDeceleration);

        double velocity = Math.min(Math.min(prevVelocity + maxAcceleration * dt, maxVelocityToEnd), maxVelocity);

        double curvature = parametric.getCurvature(closestPointT);
        double maxCurvatureVelocity = maxVelocityFromCurvature(curvature);
        
        if(Math.abs(velocity - maxCurvatureVelocity) < maxDeceleration) {
            velocity = Math.min(velocity, maxCurvatureVelocity);
        }

        prevRobotPosition = robotPosition;
        prevVelocity = velocity;

        return PurePursuitController.purePursuit(robotPosition, lookaheadPoint, velocity, trackwidth);
    }

    public double maxVelocityFromDistance(double distance, double endVelocity, double maxDeceleration) {
        //vf^2 = vi^2 + 2ad, solve for vi (deceleration = -a)
        if(distance > 0) return Math.sqrt(endVelocity * endVelocity + 2 * maxDeceleration * distance);
        else return 0;
    }

    public double maxVelocityFromCurvature(double curvature) {
        return curvature * maxAngularVelocity;
    }

    public double minDistanceToSlowdown(double curVelocity, double endVelocity, double maxDeceleration) {
        //vf^2 = vi^2 + 2ad, solve for d (deceleration = -a)
        return (curVelocity * curVelocity - endVelocity * endVelocity) / (2 * maxDeceleration);
    }

}
