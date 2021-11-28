package math;

public class Point2D {
    private double x;
    private double y;

    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point2D() {
        this(0, 0);
    }

    public Point2D(Vector2D v) {
        this(v.getX(), v.getY());
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double distance(Point2D other) {
        return Math.sqrt((other.getX() - x) * (other.getX() - x) + (other.getY() - y) * (other.getY() - y));
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public void print() {
        System.out.println("(" + x + ", " + y + ")");
    }
}
