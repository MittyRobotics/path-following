package splines;

import math.Angle;
import math.Point2D;
import math.Pose2D;

public class Parametric {
    public Point2D getPoint(double t) {
        return new Point2D();
    }

    public Angle getAngle(double t) {
        return new Angle();
    }

    public Pose2D getPose(double t) {
        return new Pose2D();
    }

    public Point2D getDerivative(double t, int n) {
        return new Point2D();
    }

    public double getGaussianQuadratureLength(double start, double end) {
        double[][] coefficients = new double[][] {
                {0.0000000000000000, 0.2729250867779006},
                {-0.2695431559523450, 0.2628045445102467},
                {0.2695431559523450, 0.2628045445102467},
                {-0.5190961292068118, 0.2331937645919905},
                {0.5190961292068118, 0.2331937645919905},
                {-0.7301520055740494, 0.1862902109277343},
                {0.7301520055740494, 0.1862902109277343},
                {-0.8870625997680953, 0.1255803694649046},
                {0.8870625997680953, 0.1255803694649046},
                {-0.9782286581460570, 0.0556685671161737},
                {0.9782286581460570, 0.0556685671161737}
        };

        double half = (end - start) / 2.0;
        double avg = (start + end) / 2.0;
        double length = 0;
        for (int i = 0; i < 11; i++) {
            length += getDerivative(avg + half * coefficients[i][0], 1).magnitude() * coefficients[i][1];
        }
        return length * half;
    }

    public double getGaussianQuadratureLength(double end) {
        return getGaussianQuadratureLength(0, end);
    }

    public double getGaussianQuadratureLength() {
        return getGaussianQuadratureLength(1.0);
    }

    public double getAccelerationMagnitudeFromCurvature(double curvature, double velocityMagnitude) {
        //a_rad = |V|^2 / R, curvature = 1/R
        return curvature * velocityMagnitude * velocityMagnitude;
    }

    public double getRawLength(double start, double end, double steps) {
        double stepSize = (end - start) / steps;
        double length = 0;

        for(double i = start; i <= end; i += stepSize) {
            length += getPoint(i).distance(getPoint(i+stepSize));
        }

        return length;
    }

}
