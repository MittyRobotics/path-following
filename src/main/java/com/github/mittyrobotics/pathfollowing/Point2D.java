package com.github.mittyrobotics.pathfollowing;

public class Point2D {
    /**
     * The x and y coordinates of the point
     */
    public double x;
    public double y;

    /**
     * Create a new point based on x and y coordinates
     * @param x x coordinate in meters
     * @param y y coordinate in meters
     */
    public Point2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Create a new point of (0, 0)
     */
    public Point2D() {
        this(0, 0);
    }

    /**
     * Create a point from a {@link Vector2D}
     * @param v {@link Vector2D} to create point with
     */
    public Point2D(Vector2D v) {
        this(v.x, v.y);
    }

    /**
     * Returns the x coordinate of the point
     * @return the x coordinate of the point
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y coordinate of the point
     * @return the y coordinate of the point
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the distance from this point to another {@link Point2D}
     * @param other {@link Point2D} to get distance from
     * @return the distance from this point to another {@link Point2D}
     */
    public double distance(Point2D other) {
        //sqrt((x1-x2)^2 + (y1-y2)^2)
        return Math.sqrt((other.x - x) * (other.x - x) + (other.y - y) * (other.y - y));
    }

    /**
     * Multiply this point by a scalar value
     * @param m the value to multiply x and y coordinates by
     * @return a new {@link Point2D} with coordinates (x*m, y*m)
     */
    public Point2D multiply(double m) {
        return new Point2D(x * m, y * m);
    }

    /**
     * Add another {@link Point2D} to this point
     * @param other {@link Point2D} to add
     * @return a new {@link Point2D} with the sum of the two points
     */
    public Point2D add(Point2D other) {
        return new Point2D(this.x + other.x, this.y + other.y);
    }

    /**
     * Returns the magnitude (distance from origin) of the point
     * @return the magnitude (distance from origin) of the point
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns a string representation of the point as (x, y) in inches
     * @return a string representation of the point as (x, y) in inches
     */
    public String toString() {
        return "(" + x * PurePursuitPath.TO_INCHES + ", " + y * PurePursuitPath.TO_INCHES + ")";
    }

    /**
     * Returns a string representation of the point as (x, y) in meters
     * @return a string representation of the point as (x, y) in meters
     */
    public String toStringMetric() {
        return "(" + x + ", " + y + ")";
    }
}
