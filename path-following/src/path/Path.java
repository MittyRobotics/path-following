package path;

import math.Point2D;
import math.Vector2D;
import splines.Parametric;

public class Path {
    private Parametric parametric;
    private double maxAcceleration, maxVelocity, startVelocity, endVelocity, maxDeceleration, maxAngularAcceleration, totalDistance, traveledDistance;

    public Path(Parametric parametric, double maxAcceleration, double maxDeceleration, double maxVelocity, double maxAngularAcceleration, double startVelocity, double endVelocity) {
        this.parametric = parametric;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxVelocity = maxVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
        this.startVelocity = 0;
        this.endVelocity = 0;
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity) {
        this(parametric, maxAcceleration, maxAcceleration, maxVelocity, Double.POSITIVE_INFINITY, startVelocity, endVelocity);
    }

    public Path(Parametric parametric, double maxAcceleration, double maxVelocity) {
        this(parametric, maxAcceleration, maxVelocity, 0, 0);
    }

    public double findClosestPointOnSpline(Point2D point, double threshold, int steps, int iterations) {

        Vector2D cur_min = new Vector2D(Double.POSITIVE_INFINITY, 0);

        for(double i = 0; i <= 1; i += 1./steps) {
            double cur_t = i;
            Vector2D derivs = getDerivsAtT(cur_t, point);
            double dt = derivs.getX() / derivs.getY();

            int counter = 0;

            while(Math.abs(dt) >= threshold && counter < iterations) {
                cur_t -= dt;
                derivs = getDerivsAtT(cur_t, point);
                dt = derivs.getX() / derivs.getY();
                counter++;
            }

            if(counter < iterations) {
                double cur_d = getDistanceAtT(cur_t, point);

                if(cur_d < cur_min.getX()) {
                    cur_min = new Vector2D(cur_d, cur_t);
                }
            }
        }

        return cur_min.getY();

    }

    public Vector2D getDerivsAtT(double t, Point2D point) {
        Point2D p = parametric.getPoint(t);
        Point2D d1 = parametric.getDerivative(t, 1);
        Point2D d2 = parametric.getDerivative(t, 2);

        double x_a = p.getX() - point.getX();
        double y_b = p.getY() - point.getY();

        return new Vector2D(
                2*(x_a*d1.getX() + y_b*d1.getY()),
                2*(d1.getX() * d1.getX() + x_a*d2.getX() + d1.getY() * d1.getY() + y_b * d2.getY())
        );
    }

    public double getDistanceAtT(double t, Point2D point) {
        Point2D p = parametric.getPoint(t);
        return (p.getX() - point.getX())*(p.getX() - point.getX()) +
                (p.getY() - point.getY())*(p.getY() - point.getY());
    }

}
