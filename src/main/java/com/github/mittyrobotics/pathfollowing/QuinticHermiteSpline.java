package com.github.mittyrobotics.pathfollowing;

public class QuinticHermiteSpline extends Parametric {
    //https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf

    /**
     * Starting and ending poses, velocities, and accelerations
     */
    private Pose2D pose0, pose1;
    private Vector2D velocity0, velocity1;
    private Vector2D acceleration0, acceleration1;

    /**
     * Create a new Quintic Hermite Spline based on starting and ending poses, velocities, and accelerations
     * @param pose0 starting {@link Pose2D}
     * @param pose1 ending {@link Pose2D}
     * @param velocity0 starting {@link Vector2D} of velocity
     * @param velocity1 ending {@link Vector2D} of velocity
     * @param acceleration0 starting {@link Vector2D} of acceleration
     * @param acceleration1 ending {@link Vector2D} of acceleration
     */
    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1, Vector2D velocity0, Vector2D velocity1,
                                Vector2D acceleration0, Vector2D acceleration1) {
        this.pose0 = pose0;
        this.pose1 = pose1;
        this.velocity0 = velocity0;
        this.velocity1 = velocity1;
        this.acceleration0 = acceleration0;
        this.acceleration1 = acceleration1;

        this.length = getGaussianQuadratureLength(17);

    }

    /**
     * Create a new Quintic Hermite Spline with accelerations of 0
     * @param pose0 starting {@link Pose2D}
     * @param pose1 ending {@link Pose2D}
     * @param velocity0 starting {@link Vector2D} of velocity
     * @param velocity1 ending {@link Vector2D} of velocity
     */
    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1, Vector2D velocity0, Vector2D velocity1) {
        this(pose0, pose1, velocity0, velocity1, new Vector2D(), new Vector2D());
    }

    /**
     * Create a new Quintic Hermite Spline with accelerations of 0 and velocities based on starting and ending poses
     * @param pose0 starting {@link Pose2D}
     * @param pose1 ending {@link Pose2D}
     */
    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1) {
        this(pose0, pose1,
                new Vector2D(pose0.getAngle(), pose0.distance(pose1)), //velocities are simply distances between poses
                new Vector2D(pose1.getAngle(), pose1.distance(pose0)));
    }

    /**
     * Create a new Quintic Hermite Spline with accelerations that create a certain curvature at the start and end
     * @param pose0 starting {@link Pose2D}
     * @param pose1 ending {@link Pose2D}
     * @param curvature0 desired curvature at start
     * @param curvature1 desired curvature at end
     */
    public QuinticHermiteSpline(Pose2D pose0, Pose2D pose1, double curvature0, double curvature1) {
        this(pose0, pose1);
        this.acceleration0 = new Vector2D(pose0.getAngle(),
                getAccelerationMagnitudeFromCurvature(curvature0, pose0.distance(pose1)));
        this.acceleration1 = new Vector2D(pose1.getAngle(),
                getAccelerationMagnitudeFromCurvature(curvature1, pose1.distance(pose0)));
    }

    /**
     * Returns the {@link Pose2D} on the spline associated with the t parameter
     * @param t t to get associated {@link Point2D}
     * @return the {@link Pose2D} on the spline associated with the t parameter
     */
    @Override
    public Point2D getPoint(double t) {
        if(t >= 0 && t <= 1) {
            //basis functions: https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf page 9-4
            double h0 = -6 * t * t * t * t * t + 15 * t * t * t * t - 10 * t * t * t + 1;
            double h1 = -3 * t * t * t * t * t + 8 * t * t * t * t - 6 * t * t * t + t;
            double h2 = -(t * t * t * t * t) / 2 + 3 * t * t * t * t / 2 - 3 * t * t * t / 2 + t * t / 2;
            double h3 = t * t * t * t * t / 2 - t * t * t * t + t * t * t / 2;
            double h4 = -3 * t * t * t * t * t + 7 * t * t * t * t - 4 * t * t * t;
            double h5 = 6 * t * t * t * t * t - 15 * t * t * t * t + 10 * t * t * t;

            return getPointFromCoefficients(h0, h1, h2, h3, h4, h5);

        } else if (t < 0) {
            return pose0.getPosition();
        } else {
            return pose1.getPosition();
        }
    }

    /**
     * Returns the {@link Angle} of the spline at a t parameter
     * @param t t to get associated {@link Angle}
     * @return the {@link Angle} of the spline at a t parameter
     */
    @Override
    public Angle getAngle(double t) {
        return new Angle(getDerivative(t, 1));
    }

    /**
     * Returns the {@link Pose2D} of the spline at a t parameter
     * @param t t to get associated {@link Pose2D}
     * @return the {@link Pose2D} of the spline at a t parameter
     */
    @Override
    public Pose2D getPose(double t) {
        return new Pose2D(getPoint(t), getAngle(t));
    }

    /**
     * Returns the n-th derivative of the spline at a t parameter as a {@link Point2D}
     * @param t t to get associated nth derivative
     * @param n degree of the derivative
     * @return {@link Point2D} representing the n-th derivative of the spline at a t parameter
     */
    @Override
    public Point2D getDerivative(double t, int n) {
        switch(n) {
            //derivatives of basis functions: https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf page 9-4
            case 1:
                double h0 = -30 * t * t * t * t + 60 * t * t * t - 30 * t * t;
                double h1 = -15 * t * t * t * t + 32 * t * t * t - 18 * t * t + 1;
                double h2 = -(5 * t * t * t * t) / 2 + 6 * t * t * t - 9 * t * t / 2 + t;
                double h3 = 5 * t * t * t * t / 2 - 4 * t * t * t + 3 * t * t / 2;
                double h4 = -15 * t * t * t * t + 28 * t * t * t - 12 * t * t;
                double h5 = 30 * t * t * t * t - 60 * t * t * t + 30 * t * t;

                return getPointFromCoefficients(h0, h1, h2, h3, h4, h5);
            case 2:
                h0 = -120 * t * t * t + 180 * t * t - 60 * t;
                h1 = -60 * t * t * t + 96 * t * t - 36 * t;
                h2 = -10 * t * t * t + 18 * t * t - 9 * t + 1;
                h3 = t * (10 * t * t - 12 * t + 3);
                h4 = -60 * t * t * t + 84 * t * t - 24 * t;
                h5 = 120 * t * t * t - 180 * t * t + 60 * t;

                return getPointFromCoefficients(h0, h1, h2, h3, h4, h5);
            default:
                return new Point2D(0, 0);
        }
    }

    /**
     * Returns a {@link Point2D} based on coefficients from origin functions
     * @param h0 coefficient for starting pose
     * @param h1 coefficient for ending pose
     * @param h2 coefficient for starting velocity
     * @param h3 coefficient for ending velocity
     * @param h4 coefficient for starting acceleration
     * @param h5 coefficient for ending acceleration
     * @return a {@link Point2D} based on coefficients from origin functions
     */
    public Point2D getPointFromCoefficients(double h0, double h1, double h2, double h3, double h4, double h5) {
        return new Point2D(
                h0 * pose0.getPosition().getX() + h1 * velocity0.getX() + h2 * acceleration0.getX() +
                        h3 * acceleration1.getX() + h4 * velocity1.getX() + h5 * pose1.getPosition().getX(),
                h0 * pose0.getPosition().getY() + h1 * velocity0.getY() + h2 * acceleration0.getY() +
                        h3 * acceleration1.getY() + h4 * velocity1.getY() + h5 * pose1.getPosition().getY());
    }

    /**
     * Returns the ending {@link Pose2D}
     * @return the ending {@link Pose2D}
     */
    public Pose2D getPose1() {return pose1;}

    /**
     * Returns the ending velocity as a {@link Vector2D}
     * @return the ending velocity as a {@link Vector2D}
     */
    public Vector2D getVelocity1() {return velocity1;}

    /**
     * Returns the ending acceleration as a {@link Vector2D}
     * @return the ending acceleration as a {@link Vector2D}
     */
    public Vector2D getAcceleration1() {return acceleration1;}

    /**
     * Returns the starting {@link Pose2D}
     * @return the starting {@link Pose2D}
     */
    public Pose2D getPose0() {return pose0;}

    /**
     * Returns the starting velocity as a {@link Vector2D}
     * @return the starting velocity as a {@link Vector2D}
     */
    public Vector2D getVelocity0() {return velocity0;}

    /**
     * Returns the starting acceleration as a {@link Vector2D}
     * @return the starting acceleration as a {@link Vector2D}
     */
    public Vector2D getAcceleration0() {return acceleration0;}

    /**
     * Sets the starting pose to the {@link Pose2D} parameter
     * @param pose new starting {@link Pose2D}
     */
    public void setPose0(Pose2D pose) {
        this.pose0 = pose;
        this.length = getGaussianQuadratureLength(17);
    }

    /**
     * Sets the starting velocity to the {@link Vector2D} parameter
     * @param velocity new starting {@link Vector2D} velocity
     */
    public void setVelocity0(Vector2D velocity) {
        this.velocity0 = velocity;
        this.length = getGaussianQuadratureLength(17);
    }

    /**
     * Sets the starting acceleration to the {@link Vector2D} parameter
     * @param acceleration new starting {@link Vector2D} acceleration
     */
    public void setAcceleration0(Vector2D acceleration) {
        this.acceleration0 = acceleration;
        this.length = getGaussianQuadratureLength(17);
    }

    /**
     * Sets the ending pose to the {@link Pose2D} parameter
     * @param pose new ending {@link Pose2D}
     */
    public void setPose1(Pose2D pose) {
        this.pose1 = pose;
        this.length = getGaussianQuadratureLength(17);
    }

    /**
     * Sets the ending velocity to the {@link Vector2D} parameter
     * @param velocity new ending {@link Vector2D} velocity
     */
    public void setVelocity1(Vector2D velocity) {
        this.velocity1 = velocity;
        this.length = getGaussianQuadratureLength(17);
    }

    /**
     * Sets the ending acceleration to the {@link Vector2D} parameter
     * @param acceleration new ending {@link Vector2D} acceleration
     */
    public void setAcceleration1(Vector2D acceleration) {
        this.acceleration1 = acceleration;
        this.length = getGaussianQuadratureLength(17);
    }

}
