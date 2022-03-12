package com.github.mittyrobotics.pathfollowing;

import java.util.ArrayList;

public class QuinticHermiteSplineGroup extends Parametric {

    //list of splines
    private ArrayList<QuinticHermiteSpline> splines = new ArrayList<>();

    /**
     * Creates a new spline group with an initial {@link QuinticHermiteSpline}
     * @param initialSpline {@link QuinticHermiteSpline} to initialize with
     */
    public QuinticHermiteSplineGroup(QuinticHermiteSpline initialSpline) {
        length = initialSpline.getLength();
        splines.add(initialSpline);
    }

    /**
     * Creates a new spline group with an initial {@link ArrayList} of {@link QuinticHermiteSpline}s
     * @param splines initial {@link ArrayList} of {@link QuinticHermiteSpline}s
     */
    public QuinticHermiteSplineGroup(ArrayList<QuinticHermiteSpline> splines) {
        this.splines = splines;
        for(QuinticHermiteSpline spline : splines) {
            length += spline.getLength();
        }
    }

    /**
     * Creates a new empty spline group
     */
    public QuinticHermiteSplineGroup() {
        length = 0;
    }

    /**
     * Adds a {@link QuinticHermiteSpline} to the list of splines
     * @param spline {@link QuinticHermiteSpline} to add
     */
    public void addSpline(QuinticHermiteSpline spline) {
        length += spline.getLength();
        splines.add(spline);
    }

    /**
     * Adds a {@link QuinticHermiteSpline} to the list of splines at the given index
     * @param index index to add the spline at
     * @param spline {@link QuinticHermiteSpline} to add
     */
    public void addSpline(int index, QuinticHermiteSpline spline) {
        length += spline.getLength();
        splines.add(index, spline);
    }

    /**
     * Get the {@link QuinticHermiteSpline} at the index of the list
     * @param index index to get spline at
     * @return {@link QuinticHermiteSpline} at the index of the list
     */
    public QuinticHermiteSpline getSpline(int index) {
        return splines.get(index);
    }

    /**
     * Removes a {@link QuinticHermiteSpline} from the list
     * @param spline {@link QuinticHermiteSpline} to remove
     */
    public void removeSpline(QuinticHermiteSpline spline) {
        if(splines.contains(spline)) {
            length -= spline.getLength();
            splines.remove(spline);
        }
    }

    /**
     * Removes the {@link QuinticHermiteSpline} at the given index
     * @param index index to remove the {@link QuinticHermiteSpline}
     */
    public void removeSpline(int index) {
        length -= splines.get(index).getLength();
        splines.remove(index);
    }

    /**
     * Gets the list of spline
     * @return an {@link ArrayList} of {@link QuinticHermiteSpline}s
     */
    public ArrayList<QuinticHermiteSpline> getSplines() {
        return splines;
    }

    /**
     * Returns the index of {@link QuinticHermiteSpline} associated with the given t
     * @param t t value to get spline of
     * @return the index of {@link QuinticHermiteSpline} associated with the given t
     */
    public int getSplineFromT(double t) {
        if(t < 0) return 0;
        if(t >= 1) return splines.size() - 1;
        return (int) (t * splines.size());
    }

    /**
     * Returns t for the spline given the overall t and the index of the spline
     * @param t overall t parameter
     * @param splineIndex index of the spline in array
     * @return t for the spline given the overall t and the index of the spline
     */
    public double getSplineTFromT(double t, int splineIndex) {
        if(t < 0) return t * splines.size();
        if(t >= 1) return 1 + (t-1) * splines.size();
        return (t - (double) splineIndex / splines.size()) * splines.size();
    }

    /**
     * Returns the {@link Point2D} at t
     * @param t t to get associated {@link Point2D}
     * @return {@link Point2D} at t
     */
    @Override
    public Point2D getPoint(double t) {
        int index = getSplineFromT(t);
        return splines.get(index).getPoint(getSplineTFromT(t, index));
    }

    /**
     * Returns the {@link Angle} at t
     * @param t t to get associated {@link Angle}
     * @return {@link Angle} at t
     */
    @Override
    public Angle getAngle(double t) {
        int index = getSplineFromT(t);
        return splines.get(index).getAngle(getSplineTFromT(t, index));
    }

    /**
     * Returns the {@link Pose2D} at t
     * @param t t to get associated {@link Pose2D}
     * @return {@link Pose2D} at t
     */
    @Override
    public Pose2D getPose(double t) {
        int index = getSplineFromT(t);
        return splines.get(index).getPose(getSplineTFromT(t, index));
    }

    /**
     * Returns the nth derivative as a {@link Point2D} at t
     * @param t t to get associated nth derivative
     * @param n degree of the derivative
     * @return the nth derivative as a {@link Point2D} at t
     */
    @Override
    public Point2D getDerivative(double t, int n) {
        int index = getSplineFromT(t);
        return splines.get(index).getDerivative(getSplineTFromT(t, index), n);
    }

    /**
     * Returns the curvature as a {@link Pose2D} at t
     * @param t t parameter to get curvature of
     * @return the curvature as a {@link Pose2D} at t
     */
    public double getCurvature(double t) {
        int index = getSplineFromT(t);
        return splines.get(index).getCurvature(getSplineTFromT(t, index));
    }

    /**
     * Returns the closest associated t value on the spline from a {@link Point2D} using Newton's method on the distance function
     * @param point the {@link Point2D} that to get closest point from
     * @param steps the number of steps to start Newton's method from
     * @param iterations the number of iterations to run Newton's method on a single step
     * @return the closest associated t value on the spline from a {@link Point2D} using Newton's method on the distance function
     */
    @Override
    public double findClosestPointOnSpline(Point2D point, int steps, int iterations) {
        Vector2D cur_min = new Vector2D(Double.POSITIVE_INFINITY, 0);
        double i = 0;
        for(QuinticHermiteSpline spline : splines) {
            double t = spline.findClosestPointOnSpline(point, steps, iterations);
            if(spline.getPoint(t).distance(point) < cur_min.getX()) {
                cur_min = new Vector2D(spline.getPoint(t).distance(point), i + t / splines.size());
            }
            i += 1. / splines.size();
        }
        return cur_min.getY();

    }

    /**
     * Returns the Gaussian quadrature length of the parametric from a start to end t parameter
     * @param start t parameter to start length calculation
     * @param end t parameter to end length calculation
     * @param steps number of steps (degree) of quadrature
     * @return the Gaussian quadrature length of the parametric from a start to end t parameter
     */
    public double getGaussianQuadratureLength(double start, double end, int steps) {
        double length = 0;
        int startSpline = getSplineFromT(start);
        int endSpline = getSplineFromT(end);
        if(startSpline == endSpline) {
            length += splines.get(startSpline).getGaussianQuadratureLength(getSplineTFromT(start, startSpline),
                    getSplineTFromT(end, endSpline), steps);
        } else {
            length += splines.get(startSpline).getGaussianQuadratureLength(getSplineTFromT(start, startSpline), 1, steps);
            length += splines.get(endSpline).getGaussianQuadratureLength(0, getSplineTFromT(end, endSpline), steps);
        }
        for (int i = startSpline + 1; i < endSpline; i++) {
            length += splines.get(i).getLength();
        }
        return length;
    }

    /**
     * Returns the t parameter associated with a certain length from the beginning
     * @param length length to get the t parameter of
     * @return the t parameter associated with a certain length from the beginning
     */
    @Override
    public double getTFromLength(double length) {
        double t = 0;
        double l = 0;
        for(QuinticHermiteSpline spline : splines) {
            if(spline.getLength() + l <= length) {
                l += spline.getLength();
                t += 1. / splines.size();
            } else {
                t += spline.getTFromLength(length - l) / splines.size();
                break;
            }
        }
        return t;
    }

    /**
     * Returns the index of the spline closest to a {@link Point2D}
     * @param point {@link Point2D} to get closest spline to
     * @param newtonsSteps the number of steps to start Newton's method from
     * @return the index of the spline closest to a {@link Point2D}
     */
    public int getIndexOfSplineFromPoint(Point2D point, int newtonsSteps) {
        return getSplineFromT(findClosestPointOnSpline(point, newtonsSteps, 5));
    }

    /**
     * Return a new {@link QuinticHermiteSplineGroup} path to this spline group's setpoint from a position, velocity, and acceleration
     * @param newPos {@link Pose2D} to start from
     * @param newVel {@link Vector2D} velocity to start from
     * @param newAcc {@link Vector2D} acceleration to start from
     * @return a new {@link QuinticHermiteSplineGroup} path to this spline group's setpoint from a position, velocity, and acceleration
     */
    @Override
    public QuinticHermiteSplineGroup getNewPath(Pose2D newPos, Vector2D newVel, Vector2D newAcc) {
        int index = getIndexOfSplineFromPoint(newPos.getPosition(), 100);
        QuinticHermiteSplineGroup group = new QuinticHermiteSplineGroup(new QuinticHermiteSpline(newPos, splines.get(index).getPose1()));
        for(int i = index + 1; i < splines.size(); i++) {
            group.addSpline(splines.get(i));
        }
        return group;
    }

    /**
     * Updates the total spline length when one {@link QuinticHermiteSpline} is edited
     * @param index index of edited {@link QuinticHermiteSpline}
     * @param prevLength previous length of edited {@link QuinticHermiteSpline}
     */
    public void updateSplineLength(int index, double prevLength) {
        length -= prevLength;
        length += splines.get(index).getLength();
    }

    /**
     * Updates the total spline length
     */
    public void updateSplineLength() {
        length = 0;
        for(QuinticHermiteSpline spline : splines) {
            length += spline.getLength();
        }
    }
}
