package path;

import math.Pose2D;
import math.Vector2D;
import splines.Parametric;

public class RamsetePath extends Path {
    protected Pose2D desiredPose;

    public RamsetePath(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularVelocity, double startVelocity, double endVelocity) {
        super(parametric, maxAcceleration, maxDeceleration, maxVelocity, maxAngularVelocity, startVelocity, endVelocity);
    }

    public RamsetePath(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    public RamsetePath(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    public DifferentialDriveState update(Pose2D robotPose, double dt, double end_threshold, double adjust_threshold, int newtonsSteps, double b, double Z, double trackwidth) {
        closestPointT = parametric.findClosestPointOnSpline(robotPose.getPosition(), 0.01, newtonsSteps, 10);
        distanceTraveled = parametric.getGaussianQuadratureLength(closestPointT, 11);

        desiredPose = parametric.getPose(closestPointT);

        velocity = Math.min(prevVelocity + maxAcceleration * dt, maxVelocity);

        double previewDistance = distanceToSlowdown(prevVelocity, 0, maxDeceleration);
        double previewT = parametric.getTFromLength(distanceTraveled + previewDistance);
        double maxVelocityAtPreview = maxVelocityFromT(previewT);

        previewVelocities.add(new Vector2D(maxVelocityAtPreview, distanceTraveled + previewDistance));
        clearOldPreviewed();

        double maxVelocityFromPreviews = getMaxVelocityFromPreviews();

        velocity = Math.min(maxVelocityFromPreviews, velocity);

        if(Double.isFinite(getRadius(closestPointT))) {
            double maxCurvatureVelocity = maxVelocityFromRadius(getRadius(closestPointT));

            velocity = Math.min(velocity, maxCurvatureVelocity);
        }

        distanceToEnd = parametric.getLength() - distanceTraveled - (velocity * dt) - end_threshold;

        maxVelocityToEnd = maxVelocityFromDistance(distanceToEnd, endVelocity, maxDeceleration);

        velocity = Math.min(velocity, maxVelocityToEnd);

        velocity = Math.max(prevVelocity - maxDeceleration * dt, velocity);

        prevVelocity = velocity;

        if(parametric.getPoint(closestPointT).distance(robotPose.getPosition()) > adjust_threshold) {
            Vector2D curVel = new Vector2D(velocity * robotPose.getAngle().cos(), velocity * robotPose.getAngle().sin());
            double acc = (velocity - prevVelocity) / dt;
            Vector2D curAcc = new Vector2D(acc * robotPose.getAngle().cos(), acc * robotPose.getAngle().sin());

            parametric = parametric.getNewPath(robotPose, curVel, curAcc);

            distanceToEnd = parametric.getLength();

        }

        return RamseteController.ramsete(robotPose, desiredPose, velocity, velocity * getCurvature(closestPointT), b, Z, trackwidth);

    }

    public double getRadius(double t) {
        return 1/getCurvature(t);
    }
}
