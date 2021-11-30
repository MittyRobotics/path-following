package path;

import math.Circle;
import math.Point2D;
import math.Pose2D;
import splines.Parametric;

public class Path {
    private Parametric parametric;
    private double maxAcceleration, maxVelocity, startVelocity, endVelocity, maxDeceleration, maxAngularVelocity;

    private double prevVelocity, distanceTraveled;

    public Path(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        this.parametric = parametric;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.startVelocity = 0;
        this.endVelocity = 0;

        this.prevVelocity = startVelocity;
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    public DifferentialDriveState update(Pose2D robotPose, double dt, double lookahead, double trackwidth) {
        double closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, 10, 10);
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 11);


        double lookaheadT = Math.min(closestPointT + lookahead, 1.0);
        Point2D lookaheadPoint = parametric.getPoint(lookaheadT);


        double distanceToEnd = parametric.getLength() - distanceTraveled;
        double maxVelocityToEnd = maxVelocityFromDistance(distanceToEnd, endVelocity, maxDeceleration);


        double velocity = Math.min(Math.min(prevVelocity + maxAcceleration * dt, maxVelocityToEnd), maxVelocity);


//        double maxCurvatureVelocity = maxVelocityFromCurvature(parametric.getCurvature(closestPointT));
        Circle tangentCircle = new Circle();
        tangentCircle.updateFromPoseAndPoint(robotPose, lookaheadPoint);
        double purePursuitRadius = tangentCircle.getRadius();


        if(Double.isFinite(purePursuitRadius)) {
            double maxCurvatureVelocity = maxVelocityFromCurvature(1 / purePursuitRadius);
            velocity = Math.min(velocity, maxCurvatureVelocity);
        }


        prevVelocity = velocity;

        return PurePursuitController.purePursuit(purePursuitRadius, velocity, trackwidth);
    }

    public double maxVelocityFromDistance(double distance, double endVelocity, double maxDeceleration) {
        //vf^2 = vi^2 + 2ad, solve for vi (deceleration = -a)
        if(distance > 0) return Math.sqrt(endVelocity * endVelocity + 2 * maxDeceleration * distance);
        else return 0;
    }

    public double maxVelocityFromCurvature(double curvature) {
        if(Double.isInfinite(maxAngularVelocity)) return Double.POSITIVE_INFINITY;
        else return curvature * maxAngularVelocity;
    }

    public boolean isFinished(Pose2D robotPosition, double threshold) {
        return robotPosition.getPosition().distance(parametric.getPoint(1.0)) <= threshold;
    }

}
