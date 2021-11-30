package math;

public class Circle {
    double radius;
    Point2D center;

    public Circle() {}

    public Circle(double radius, Point2D center) {
        this.radius = radius;
        this.center = center;
    }

    public void updateFromPoseAndPoint(Pose2D pose, Point2D other) {
        Angle angleOfRadius = new Angle(pose.getAngle() - Math.PI/2);
        Line radius1 = new Line(pose.getPosition(), angleOfRadius);

        Line lineThroughPoints = new Line(pose.getPosition(), other);

        if(lineThroughPoints.getSlope() == new Angle(pose.getAngle()).tan()) {
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
