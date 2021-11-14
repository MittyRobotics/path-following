package math;

public class Angle {
    private double radians;

    public Angle(double x, double y) {
        radians = Math.atan2(y, x);
    }

    public Angle(double radians) {
        this.radians = radians;
    }

    public double tan() {
        return Math.tan(radians);
    }

    public double sin() {
        return Math.sin(radians);
    }

    public double cos() {
        return Math.cos(radians);
    }

    public double getAngle() {
        return radians;
    }
}
