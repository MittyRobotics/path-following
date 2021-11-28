package math;

public class Line {
    private double slope, y_int;

    public Line(double slope, double y_int) {
        this.slope = slope;
        this.y_int = y_int;
    }

    public Line(Point2D point, Angle angle) {
        this.slope = angle.tan();

        this.y_int = point.getY() - point.getX() * this.slope;
    }

    public Line(Point2D point, double slope) {
        this.slope = slope;
        this.y_int = point.getY() - point.getX() * this.slope;
    }

    public Line(Point2D point1, Point2D point2) {
        this(point1, (point2.getY() - point1.getY())/(point2.getX() - point1.getX()));
    }

    public double getSlope() {
        return slope;
    }

    public double getY_int() {
        return y_int;
    }

    public Point2D getIntersection(Line other) {
        double x = (other.y_int - this.y_int) / (this.slope - other.slope);
        double y = slope * x + y_int;

        return new Point2D(x, y);
    }

    public Line getTangentAtPoint(Point2D other) {
        return new Line(other, -1./slope);
    }

    public void print() {
        System.out.println("m=" + slope + ", b=" + y_int);
    }
}
