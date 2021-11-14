package splines;

import math.Angle;
import math.Pose2D;
import math.Vector2D;

public class QuinticHermiteSpline {
    public Pose2D pose0, pose1;
    public Vector2D velocity0, velocity1;
    public Vector2D acceleration0, acceleration1;

    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1, Vector2D velocity0, Vector2D velocity1,
                                Vector2D acceleration0, Vector2D acceleration1) {
        this.pose0 = pose0;
        this.pose1 = pose1;
        this.velocity0 = velocity0;
        this.velocity1 = velocity1;
        this.acceleration0 = acceleration0;
        this.acceleration1 = acceleration1;
    }

    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1, Vector2D velocity0, Vector2D velocity1) {
        this(pose0, pose1, velocity0, velocity1, new Vector2D(), new Vector2D());
    }

    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1) {
        this(pose0, pose1,
                new Vector2D(new Angle(pose0.getAngle()), pose0.distance(pose1)),
                new Vector2D(new Angle(pose1.getAngle()), pose1.distance(pose0)));
    }

    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1, double curvature0, double curvature1) {
        this(pose0, pose1);
        this.acceleration0 = new Vector2D(new Angle(pose0.getAngle()),
                getAccelerationMagnitudeFromCurvature(curvature0, pose0.distance(pose1)));
        this.acceleration1 = new Vector2D(new Angle(pose1.getAngle()),
                getAccelerationMagnitudeFromCurvature(curvature1, pose1.distance(pose0)));
    }


    public double getAccelerationMagnitudeFromCurvature(double curvature, double velocityMagnitude) {
        //a_rad = |V|^2 / R, curvature = 1/R
        return curvature * velocityMagnitude * velocityMagnitude;
    }
}
