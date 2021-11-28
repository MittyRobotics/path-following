package math;

public class Vector2D {
    private Angle angle;
    private double magnitude;

    private double x;
    private double y;

    public Vector2D(Angle angle, double magnitude) {
        this.angle = angle;
        this.magnitude = magnitude;

        this.x = magnitude * angle.cos();
        this.y = magnitude * angle.sin();
    }

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;

        this.angle = new Angle(Math.atan2(y, x));
        this.magnitude = Math.sqrt(x * x + y * y);
    }

    public Vector2D() {
        this(0, 0);
    }

    public Vector2D(Point2D p) {
        this(p.getX(), p.getY());
    }

    public double getMagnitude() {
        return magnitude;
    }

    public Angle getAngle() {
        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector2D add(Vector2D other) {
        return new Vector2D(this.x + other.getX(), this.y + other.getY());
    }

    public double distance(Vector2D other) {
        return Math.sqrt((other.getX() - x) * (other.getX() - x) + (other.getY() - y) * (other.getY() - y));
    }

    public void print() {
        System.out.println("(" + x + ", " + y + ")");
    }
}
