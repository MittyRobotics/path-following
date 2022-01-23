package pathfollowing;

public class Angle {
    /**
     * Angle in radians
     */
    private double radians;

    /**
     * Creates default angle of 0 radians
     */
    public Angle() {
        this(0);
    }

    /**
     * Creates angle equal to angle from origin of a vector (x, y)
     * @param x x value of vector
     * @param y y value of vector
     */
    public Angle(double x, double y) {
        this(Math.atan2(y, x));
    }

    /**
     * Creates angle of rad radians
     * @param rad value of angle in radians
     */
    public Angle(double rad) {
        this.radians = standardize(rad);
    }

    /**
     * Creates angle equal to angle from origin of a {@link Point2D}
     * @param point {@link Point2D} to get angle from
     */
    public Angle(Point2D point) {
        this(point.getX(), point.getY());
    }


    /**
     * Returns tangent of the current angle, positive or negative infinity if {@link Angle}.cos() equals 0
     * @return tangent of the current angle, positive or negative infinity if {@link Angle}.cos() equals 0
     */
    public double tan() {
        if(Math.cos(radians) == 0) {
            return (Math.sin(radians) > 0 ? Double.POSITIVE_INFINITY : Double.NEGATIVE_INFINITY);
        }
        return Math.tan(radians);
    }

    /**
     * Adds radians to the {@link Angle}
     * @param radians radians to add
     */
    public void add(double radians) {
        this.radians = standardize(this.radians + radians);
    }

    /**
     * Returns sine of the current angle
     * @return sine of the current angle
     */
    public double sin() {
        return Math.sin(radians);
    }

    /**
     * Returns cosine of the current angle
     * @return cosine of the current angle
     */
    public double cos() {
        return Math.cos(radians);
    }

    /**
     * Returns angle in radians
     * @return angle in radians
     */
    public double getRadians() {
        return radians;
    }

    /**
     * Returns string representation of the {@link Angle} in degrees
     * @return string representation of the {@link Angle} in degrees
     */
    public String toString() {
        return radians * 180 / Math.PI + "°";
    }

    /**
     * Returns string representation of the {@link Angle} in radians
     * @return string representation of the {@link Angle} in radians
     */
    public String toStringMetric() {
        return radians + "";
    }

    /**
     * Returns angle in radians between two {@link Angle}s
     * @param other {@link Angle} to get the angle between
     * @return string representation of the {@link Angle} in degrees
     */
    public double getAngleBetween(Angle other) {
        double phi = Math.abs(this.radians - other.radians) % (2*Math.PI);
        return phi > Math.PI ? 2*Math.PI - phi : phi;
    }

    /**
     * Standardizes an angle to between [0, 2π] radians
     * @param radians radians to standardize
     * @return radians standardized to between [0, 2π]
     */
    public static double standardize(double radians) {
        if(radians > 0) return radians - (2*Math.PI) * (int)(radians / (2*Math.PI));
        else return radians + (2*Math.PI) * ((int)(radians / (2*Math.PI))+1);
    }

}
