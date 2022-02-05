package pathfollowing;

public class Line {
    /**
     * Variables to store slope and y intercept
     */
    private double slope, y_int;

    /**
     * Creates a line based on a {@link Point2D} and an {@link Angle} from the origin
     * @param point {@link Point2D} that the line passes through
     * @param angle {@link Angle} from the origin
     */
    public Line(Point2D point, Angle angle) {
        this(point, angle.tan());
    }

    /**
     * Creates a line based on a {@link Point2D} and the line's slope
     * @param point {@link Point2D} that the line passes through
     * @param slope slope of the line
     */
    public Line(Point2D point, double slope) {
        this.slope = slope;
        this.y_int = point.getY() - point.getX() * this.slope;
    }

    /**
     * Creates a line based on two {@link Point2D}s on the line
     * @param point1 first {@link Point2D} on the line
     * @param point2 second {@link Point2D} on the line
     */
    public Line(Point2D point1, Point2D point2) {
        this(point1, (point2.getY() - point1.getY())/(point2.getX() - point1.getX()));
    }

    /**
     * Returns the slope of the line
     * @return slope of the line
     */
    public double getSlope() {
        return slope;
    }

    /**
     * Returns the {@link Point2D} where the two lines intersect
     * @param other {@link Line} to find intersection with
     * @return {@link Point2D} where the two lines intersect
     */
    public Point2D getIntersection(Line other) {
        double x = (other.y_int - this.y_int) / (this.slope - other.slope);
        double y = slope * x + y_int;

        return new Point2D(x, y);
    }

    /**
     * Returns a {@link Line} tangent to this line at the given {@link Point2D}
     * @param other {@link Point2D} to find tangent line through
     * @return {@link Line} tangent to this line at the given {@link Point2D}
     */
    public Line getTangentAtPoint(Point2D other) {
        return new Line(other, -1./slope);
    }

    /**
     * Returns a string representation of the line as y=mx+b in inches
     * @return a string representation of the line as y=mx+b in inches
     */
    public String toString() {
        return "y = " + slope + "x + " + y_int * PurePursuitPath.TO_INCHES;
    }

    /**
     * Returns a string representation of the line as y=mx+b in meters
     * @return a string representation of the line as y=mx+b in meters
     */
    public String toStringMetric() {
        return "y = " + slope + "x + " + y_int;
    }
}
