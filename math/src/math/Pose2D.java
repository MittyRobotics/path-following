package math;

public class Pose2D {
    public Vector2D position;
    public Angle angle;

    public double x;
    public double y;
    public double rotation;

    public Pose2D(Vector2D position, Angle angle) {
        this.angle = angle;
        this.position = position;
        this.x = position.getX();
        this.y = position.getY();
        this.rotation = angle.getAngle();
    }

    public Pose2D(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.rotation = angle;

        this.position = new Vector2D(x, y);
        this.angle = new Angle(angle);
    }
}
