package com.github.mittyrobotics.pathfollowing;

public class Circle {
    /**
     * Radius in meters
     */
    private double radius;

    /**
     * Center as a {@link Point2D} in meters
     */
    private Point2D center;

    /**
     * Creates a circle with radius 0 and center (0, 0)
     */
    public Circle() {
        this(0, new Point2D());
    }

    /**
     * Creates a new circle
     * @param radius radius in meters
     * @param center {@link Point2D} representing center
     */
    public Circle(double radius, Point2D center) {
        this.radius = radius;
        this.center = center;
    }

    /**
     * Returns the orientation between a {@link Pose2D} and a {@link Point2D}
     * @param pose {@link Pose2D} representing starting position
     * @param point3 {@link Point2D} point to get the orientation of
     * @return 0 means straight line, 1 means point3 is to the right of pose, 2 means point3 is to the left of pose
     */
    public int orientationOfPoseAndPoint(Pose2D pose, Point2D point3) {
        Point2D point1 = pose.getPosition();
        Point2D point2 = new Point2D(point1.getX() + pose.getAngle().cos(), point1.getY() + pose.getAngle().sin());

        double test = (point2.getY() - point1.getY()) * (point3.getX() - point2.getX()) -
                   (point2.getX() - point1.getX()) * (point3.getY() - point2.getY());

        if(Math.abs(test) < 2e-9) return 0;

        return (test > 0) ? 1 : 2;
    }

    /**
     * Updates radius and center based on a tangent {@link Pose2D} and another {@link Point2D} on the circle
     * @param pose tangent {@link Pose2D}
     * @param other another {@link Point2D} on the circle
     */
    public void updateFromPoseAndPoint(Pose2D pose, Point2D other) {
        Angle angleOfRadius = new Angle(pose.getAngle().getRadians() - Math.PI/2);
        Line radius1 = new Line(pose.getPosition(), angleOfRadius);

        Line lineThroughPoints = new Line(pose.getPosition(), other);

        if(Math.abs(lineThroughPoints.getSlope() - pose.getAngle().tan()) < 2e-9) {
            this.radius = Double.POSITIVE_INFINITY;
        } else {
            Point2D midpoint = new Point2D((pose.getPosition().getX() + other.getX()) / 2, (pose.getPosition().getY() + other.getY()) / 2);
            Line radius2 = lineThroughPoints.getTangentAtPoint(midpoint);

            this.center = radius1.getIntersection(radius2);
            this.radius = center.distance(other);
        }
    }

    /**
     * Returns radius in meters
     * @return radius in meters
     */
    public double getRadius() {
        return radius;
    }

    /**
     * Returns center as a {@link Point2D} in meters
     * @return center as a {@link Point2D} in meters
     */
    public Point2D getCenter() {
        return center;
    }

    /**
     * Returns string representation of the {@link Circle} in inches
     * @return string representation of the {@link Circle} in inches
     */
    public String toString() {
        return "radius = " + radius * PurePursuitPath.TO_INCHES + ", " + "center = " + center.toString();
    }

    /**
     * Returns string representation of the {@link Circle} in meters
     * @return string representation of the {@link Circle} in meters
     */
    public String toStringMetric() {
        return "radius = " + radius + ", " + "center = " + center.toStringMetric();
    }
}
