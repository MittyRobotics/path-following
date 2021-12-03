package math;

public class Circle {
    double radius;
    Point2D center;

    public Circle() {}

    public Circle(double radius, Point2D center) {
        this.radius = radius;
        this.center = center;
    }

    public int orientationOfPoseAndPoint(Pose2D pose, Point2D point3) {
        Point2D point1 = pose.getPosition();
        Point2D point2 = new Point2D(point1.getX() + pose.getAngle().cos(), point1.getY() + pose.getAngle().sin());

        double test = (point2.getY() - point1.getY()) * (point3.getX() - point2.getX()) -
                   (point2.getX() - point1.getX()) * (point3.getY() - point2.getY());

        if(Math.abs(test) < 2e-9) return 0;

        return (test > 0) ? 1 : 2;
    }

    public void updateFromPoseAndPoint(Pose2D pose, Point2D other) {
        Angle angleOfRadius = new Angle(pose.getAngleRadians() - Math.PI/2);
        Line radius1 = new Line(pose.getPosition(), angleOfRadius);

        Line lineThroughPoints = new Line(pose.getPosition(), other);

        if(lineThroughPoints.getSlope() == new Angle(pose.getAngleRadians()).tan()) {
            this.radius = Double.POSITIVE_INFINITY;
        } else {
            Point2D midpoint = new Point2D((pose.getPosition().getX() + other.getX()) / 2, (pose.getPosition().getY() + other.getY()) / 2);
            Line radius2 = lineThroughPoints.getTangentAtPoint(midpoint);

            this.center = radius1.getIntersection(radius2);
            this.radius = center.distance(other);
        }
    }

    public double getRadius() {
        return radius;
    }

    public Point2D getCenter() {
        return center;
    }

    public void print() {
        System.out.println("radius=" + radius + ", " + "center=(" + center.getX() + ", " + center.getY() + ")");
    }
}
