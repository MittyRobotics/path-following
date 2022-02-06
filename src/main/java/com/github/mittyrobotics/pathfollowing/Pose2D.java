package com.github.mittyrobotics.pathfollowing;

public class Pose2D {
    /**
     * The {@link Point2D} and {@link Angle} associated with the pose
     */
    private Point2D position;
    private Angle angle;

    /**
     * Create a new pose with a point (0, 0) and angle 0
     */
    public Pose2D() {
        this(new Point2D(), new Angle());
    }

    /**
     * Create a new pose with the given {@link Vector2D} and {@link Angle}
     * @param position {@link Vector2D} representing position of the pose
     * @param angle {@link Angle} representing angle of the pose
     */
    public Pose2D(Vector2D position, Angle angle) {
        this.angle = angle;
        this.position = new Point2D(position);
    }

    /**
     * Create a new pose with the given {@link Point2D} and {@link Angle}
     * @param position {@link Point2D} representing position of the pose
     * @param angle {@link Angle} representing angle of the pose
     */
    public Pose2D(Point2D position, Angle angle) {
        this.angle = angle;
        this.position = position;
    }

    /**
     * Create a new pose with the given x and y coordinates and angle in radians
     * @param x x-coordinate of the position of the pose
     * @param y y-coordinate of the position of the pose
     * @param angle angle of the pose in radians
     */
    public Pose2D(double x, double y, double angle) {
        this.position = new Point2D(x, y);
        this.angle = new Angle(angle);
    }

    /**
     * Returns the distance of this pose from another {@link Pose2D}
     * @param other {@link Pose2D} pose to get distance from
     * @return the distance of this pose from another {@link Pose2D}
     */
    public double distance(Pose2D other) {
        return other.getPosition().distance(this.position);
    }

    /**
     * Returns the {@link Point2D} representing the position of the point
     * @return the {@link Point2D} representing the position of the point
     */
    public Point2D getPosition() {
        return position;
    }

    /**
     * Returns the {@link Angle} of the point
     * @return the {@link Angle} of the point
     */
    public Angle getAngle() {
        return angle;
    }

    /**
     * Returns a string representation of the pose as [(x, y), degrees] in inches
     * @return a string representation of the pose as [(x, y), degrees] in inches
     */
    public String toString() {
        return position.toString() + ", " + angle.toString();
    }

    /**
     * Returns a string representation of the pose as [(x, y), radians] in meters
     * @return a string representation of the pose as [(x, y), radians] in meters
     */
    public String toStringMetric() {
        return position.toStringMetric() + ", " + angle.toStringMetric();
    }
}
