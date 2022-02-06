package com.github.mittyrobotics.pathfollowing;

public class Vector2D {
    /**
     * The x and y values of the vector
     */
    public double x;
    public double y;

    /**
     * Create a vector from an {@link Angle} and a magnitude
     * @param angle {@link Angle} of vector
     * @param magnitude magnitude of vector
     */
    public Vector2D(Angle angle, double magnitude) {
        this.x = magnitude * angle.cos();
        this.y = magnitude * angle.sin();
    }

    /**
     * Create a new vector based on x and y coordinates
     * @param x x coordinate in meters
     * @param y y coordinate in meters
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Create a new vector of (0, 0)
     */
    public Vector2D() {
        this(0, 0);
    }

    /**
     * Returns the x value of the vector
     * @return the x value of the vector
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the y value of the vector
     * @return the y value of the vector
     */
    public double getY() {
        return y;
    }

    /**
     * Add another {@link Vector2D} to this vector
     * @param other {@link Vector2D} to add
     * @return a new vector representing the sum of this and the other vector
     */
    public Vector2D add(Vector2D other) {
        return new Vector2D(this.x + other.x, this.y + other.y);
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
