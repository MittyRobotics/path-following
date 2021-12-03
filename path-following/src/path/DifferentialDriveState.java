package path;

public class DifferentialDriveState {
    private double linearVelocity, angularVelocity, leftVelocity, rightVelocity, radius, trackWidth;

    public DifferentialDriveState(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public DifferentialDriveState(double linearVelocity, double angularVelocity, double leftVelocity, double rightVelocity, double radius, double trackWidth) {
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.radius = radius;
        this.trackWidth = trackWidth;
    }

    public void updateFromLinearAndAngularVelocity(double linearVelocity, double angularVelocity, double trackWidth) {
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
        this.trackWidth = trackWidth;

        if(Math.abs(angularVelocity) < 2e-9) {
            this.radius = Double.POSITIVE_INFINITY;
            this.leftVelocity = linearVelocity;
            this.rightVelocity = linearVelocity;
        } else {
            this.radius = linearVelocity / angularVelocity;
            this.leftVelocity = angularVelocity * (radius - trackWidth / 2.);
            this.rightVelocity = angularVelocity * (radius + trackWidth / 2.);
        }
    }

    public void updateFromLinearVelocityAndRadius(double linearVelocity, double radius, boolean turnRight, double trackWidth) {
        if(Double.isInfinite(radius)) {
            this.angularVelocity = 0;
        } else {
            this.angularVelocity = linearVelocity / radius;
        }
        if(turnRight) angularVelocity *= -1;
        updateFromLinearAndAngularVelocity(linearVelocity, this.angularVelocity, trackWidth);
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }


 }
