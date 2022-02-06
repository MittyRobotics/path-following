package com.github.mittyrobotics.pathfollowing;

public class Parametric {
    /**
     * Length of the parametric
     */
    protected double length;

    /**
     * Returns the {@link Point2D} at t
     * @param t t to get associated {@link Point2D}
     * @return {@link Point2D} at t
     */
    public Point2D getPoint(double t) {
        return new Point2D();
    }

    /**
     * Returns the {@link Angle} at t
     * @param t t to get associated {@link Angle}
     * @return {@link Angle} at t
     */
    public Angle getAngle(double t) {
        return new Angle();
    }

    /**
     * Returns the total length of the parametric
     * @return the total length of the parametric
     */
    public double getLength() {
        return length;
    }

    /**
     * Returns the {@link Pose2D} at t
     * @param t t to get associated {@link Pose2D}
     * @return {@link Pose2D} at t
     */
    public Pose2D getPose(double t) {
        return new Pose2D();
    }

    /**
     * Returns the nth derivative as a {@link Point2D} at t
     * @param t t to get associated nth derivative
     * @param n degree of the derivative
     * @return the nth derivative as a {@link Point2D} at t
     */
    public Point2D getDerivative(double t, int n) {
        return new Point2D();
    }

    /**
     * Returns an array of the nth Legendre-Gauss coefficients (first column is weights, second column is points)
     * @param n degree of the coefficients
     * @return double array of the nth Legendre-Gauss coefficients
     */
    public double[][] getCoefficients(int n) {
        //Source: https://pomax.github.io/bezierinfo/legendre-gauss.html
        switch (n) {
            case 2:
                return new double[][] {
                        {1.0000000000000000, -0.577350269189626},
                        {1.0000000000000000, 0.577350269189626}
                };
            case 3:
                return new double[][] {
                        {0.888888888888889, 0.0000000000000000},
                        {0.555555555555556, -0.774596669241483},
                        {0.555555555555556, 0.774596669241483}
                };
            case 4:
                return new double[][] {
                        {0.652145154862546, -0.339981043584856},
                        {0.652145154862546, 0.339981043584856},
                        {0.347854845137454, -0.861136311594053},
                        {0.347854845137454, 0.861136311594053}
                };
            case 5:
                return new double[][] {
                        {0.568888888888889, 0.0000000000000000},
                        {0.478628670499367, -0.538469310105683},
                        {0.478628670499367, 0.538469310105683},
                        {0.236926885056189, -0.9061798459386640},
                        {0.236926885056189, 0.9061798459386640},
                };
            case 6:
                return new double[][] {
                        {0.360761573048139, 0.661209386466265},
                        {0.360761573048139, -0.661209386466265},
                        {0.4679139345726910, -0.238619186083197},
                        {0.4679139345726910, 0.238619186083197},
                        {0.17132449237917, -0.932469514203152},
                        {0.17132449237917, 0.932469514203152}
                };
            case 7:
                return new double[][] {
                        {0.417959183673469, 0.0000000000000000},
                        {0.381830050505119, 0.405845151377397},
                        {0.381830050505119, -0.405845151377397},
                        {0.279705391489277, -0.741531185599395},
                        {0.279705391489277, 0.741531185599395},
                        {0.12948496616887, -0.949107912342759},
                        {0.12948496616887, 0.949107912342759}
                };
            case 8:
                return new double[][] {
                        {0.3626837833783620, -0.18343464249565},
                        {0.3626837833783620, 0.18343464249565},
                        {0.313706645877887, -0.5255324099163290},
                        {0.313706645877887, 0.5255324099163290},
                        {0.222381034453375, -0.796666477413627},
                        {0.222381034453375, 0.796666477413627},
                        {0.101228536290376, -0.960289856497536},
                        {0.101228536290376, 0.960289856497536}
                };
            case 9:
                return new double[][] {
                        {0.33023935500126, 0.0000000000000000},
                        {0.180648160694857, -0.836031107326636},
                        {0.180648160694857, 0.836031107326636},
                        {0.0812743883615744, -0.968160239507626},
                        {0.0812743883615744, 0.968160239507626},
                        {0.312347077040003, -0.324253423403809},
                        {0.312347077040003, 0.324253423403809},
                        {0.260610696402935, -0.61337143270059},
                        {0.260610696402935, 0.61337143270059}
                };
            case 10:
                return new double[][] {
                        {0.295524224714753, -0.148874338981631},
                        {0.295524224714753, 0.148874338981631},
                        {0.269266719309996, -0.433395394129247},
                        {0.269266719309996, 0.433395394129247},
                        {0.2190863625159820, -0.679409568299024},
                        {0.2190863625159820, 0.679409568299024},
                        {0.149451349150581, -0.865063366688985},
                        {0.149451349150581, 0.865063366688985},
                        {0.0666713443086881, -0.973906528517172},
                        {0.0666713443086881, 0.973906528517172}
                };
            case 11:
                return new double[][] {
                        {0.272925086777901, 0.0000000000000000},
                        {0.262804544510247, -0.2695431559523450},
                        {0.262804544510247, 0.2695431559523450},
                        {0.233193764591991, -0.519096129206812},
                        {0.233193764591991, 0.519096129206812},
                        {0.186290210927734, -0.730152005574049},
                        {0.186290210927734, 0.730152005574049},
                        {0.125580369464905, -0.887062599768095},
                        {0.125580369464905, 0.887062599768095},
                        {0.0556685671161737, -0.9782286581460570},
                        {0.0556685671161737, 0.9782286581460570}
                };
            case 12:
                return new double[][] {
                        {0.249147045813403, -0.125233408511469},
                        {0.249147045813403, 0.125233408511469},
                        {0.233492536538355, -0.36783149899818},
                        {0.233492536538355, 0.36783149899818},
                        {0.203167426723066, -0.587317954286618},
                        {0.203167426723066, 0.587317954286618},
                        {0.160078328543346, -0.769902674194305},
                        {0.160078328543346, 0.769902674194305},
                        {0.106939325995318, -0.904117256370475},
                        {0.106939325995318, 0.904117256370475},
                        {0.0471753363865118, -0.981560634246719},
                        {0.0471753363865118, 0.981560634246719}
                };
            case 13:
                return new double[][] {
                        {0.232551553230874, 0.0000000000000000},
                        {0.226283180262897, -0.230458315955135},
                        {0.226283180262897, 0.230458315955135},
                        {0.207816047536889, -0.448492751036447},
                        {0.207816047536889, 0.448492751036447},
                        {0.178145980761946, -0.64234933944034},
                        {0.178145980761946, 0.64234933944034},
                        {0.138873510219787, -0.80157809073331},
                        {0.138873510219787, 0.80157809073331},
                        {0.0921214998377285, -0.917598399222978},
                        {0.0921214998377285, 0.917598399222978},
                        {0.0404840047653159, -0.984183054718588},
                        {0.0404840047653159, 0.984183054718588}
                };
            case 14:
                return new double[][] {
                        {0.215263853463158, -0.108054948707344},
                        {0.215263853463158, 0.108054948707344},
                        {0.205198463721296, -0.31911236892789},
                        {0.205198463721296, 0.31911236892789},
                        {0.185538397477938, -0.515248636358154},
                        {0.185538397477938, 0.515248636358154},
                        {0.157203167158194, -0.687292904811686},
                        {0.157203167158194, 0.687292904811686},
                        {0.121518570687903, -0.8272013150697650},
                        {0.121518570687903, 0.8272013150697650},
                        {0.0801580871597602, -0.928434883663574},
                        {0.0801580871597602, 0.928434883663574},
                        {0.0351194603317519, -0.986283808696812},
                        {0.0351194603317519, 0.986283808696812}
                };
            case 15:
                return new double[][] {
                        {0.202578241925561, 0.0000000000000000},
                        {0.198431485327112, -0.201194093997435},
                        {0.198431485327112, 0.201194093997435},
                        {0.186161000015562, -0.394151347077563},
                        {0.186161000015562, 0.394151347077563},
                        {0.166269205816994, -0.570972172608539},
                        {0.166269205816994, 0.570972172608539},
                        {0.139570677926154, -0.72441773136017},
                        {0.139570677926154, 0.72441773136017},
                        {0.107159220467172, -0.848206583410427},
                        {0.107159220467172, 0.848206583410427},
                        {0.0703660474881081, -0.9372733924007060},
                        {0.0703660474881081, 0.9372733924007060},
                        {0.0307532419961173, -0.987992518020485},
                        {0.0307532419961173, 0.987992518020485}
                };
            case 16:
                return new double[][] {
                        {0.189450610455069, -0.0950125098376374},
                        {0.189450610455069, 0.0950125098376374},
                        {0.182603415044924, -0.281603550779259},
                        {0.182603415044924, 0.281603550779259},
                        {0.169156519395003, -0.458016777657227},
                        {0.169156519395003, 0.458016777657227},
                        {0.149595988816577, -0.617876244402644},
                        {0.149595988816577, 0.617876244402644},
                        {0.124628971255534, -0.7554044083550030},
                        {0.124628971255534, 0.7554044083550030},
                        {0.0951585116824928, -0.865631202387832},
                        {0.0951585116824928, 0.865631202387832},
                        {0.0622535239386479, -0.944575023073233},
                        {0.0622535239386479, 0.944575023073233},
                        {0.0271524594117541, -0.98940093499165},
                        {0.0271524594117541, 0.98940093499165}
                };
            case 17:
                return new double[][] {
                        {0.179446470356207, 0.0000000000000000},
                        {0.176562705366993, -0.178484181495848},
                        {0.176562705366993, 0.178484181495848},
                        {0.1680041021564500, -0.351231763453876},
                        {0.1680041021564500, 0.351231763453876},
                        {0.15404576107681, -0.512690537086477},
                        {0.15404576107681, 0.512690537086477},
                        {0.135136368468526, -0.657671159216691},
                        {0.135136368468526, 0.657671159216691},
                        {0.1118838471934040, -0.781514003896801},
                        {0.1118838471934040, 0.781514003896801},
                        {0.0850361483171792, -0.880239153726986},
                        {0.0850361483171792, 0.880239153726986},
                        {0.0554595293739872, -0.950675521768768},
                        {0.0554595293739872, 0.950675521768768},
                        {0.0241483028685479, -0.990575475314417},
                        {0.0241483028685479, 0.990575475314417}
                };
            case 18:
                return new double[][] {
                        {0.169142382963144, -0.0847750130417353},
                        {0.169142382963144, 0.0847750130417353},
                        {0.164276483745833, -0.251886225691506},
                        {0.164276483745833, 0.251886225691506},
                        {0.154684675126265, -0.411751161462843},
                        {0.154684675126265, 0.411751161462843},
                        {0.140642914670651, -0.559770831073948},
                        {0.140642914670651, 0.559770831073948},
                        {0.122555206711479, -0.691687043060353},
                        {0.122555206711479, 0.691687043060353},
                        {0.100942044106287, -0.803704958972523},
                        {0.100942044106287, 0.803704958972523},
                        {0.0764257302548891, -0.892602466497556},
                        {0.0764257302548891, 0.892602466497556},
                        {0.0497145488949698, -0.955823949571398},
                        {0.0497145488949698, 0.955823949571398},
                        {0.0216160135264833, -0.991565168420931},
                        {0.0216160135264833, 0.991565168420931}
                };
            case 19:
                return new double[][] {
                        {0.161054449848784, 0.0000000000000000},
                        {0.158968843393954, -0.160358645640225},
                        {0.158968843393954, 0.160358645640225},
                        {0.15276604206586, -0.31656409996363},
                        {0.15276604206586, 0.31656409996363},
                        {0.142606702173607, -0.464570741375961},
                        {0.142606702173607, 0.464570741375961},
                        {0.128753962539336, -0.6005453046616810},
                        {0.128753962539336, 0.6005453046616810},
                        {0.1115666455473340, -0.720966177335229},
                        {0.1115666455473340, 0.720966177335229},
                        {0.0914900216224500, -0.822714656537143},
                        {0.0914900216224500, 0.822714656537143},
                        {0.0690445427376412, -0.903155903614818},
                        {0.0690445427376412, 0.903155903614818},
                        {0.0448142267656996, -0.9602081521348300},
                        {0.0448142267656996, 0.9602081521348300},
                        {0.0194617882297265, -0.992406843843584},
                        {0.0194617882297265, 0.992406843843584}
                };
            case 20:
                return new double[][] {
                        {0.152753387130726, -0.0765265211334973},
                        {0.152753387130726, 0.0765265211334973},
                        {0.149172986472604, -0.227785851141645},
                        {0.149172986472604, 0.227785851141645},
                        {0.1420961093183820, -0.37370608871542},
                        {0.1420961093183820, 0.37370608871542},
                        {0.131688638449177, -0.510867001950827},
                        {0.131688638449177, 0.510867001950827},
                        {0.118194531961518, -0.6360536807265150},
                        {0.118194531961518, 0.6360536807265150},
                        {0.10193011981724, -0.746331906460151},
                        {0.10193011981724, 0.746331906460151},
                        {0.0832767415767048, -0.839116971822219},
                        {0.0832767415767048, 0.839116971822219},
                        {0.0626720483341091, -0.912234428251326},
                        {0.0626720483341091, 0.912234428251326},
                        {0.0406014298003869, -0.963971927277914},
                        {0.0406014298003869, 0.963971927277914},
                        {0.0176140071391521, -0.993128599185095},
                        {0.0176140071391521, 0.993128599185095}
                };
        };
        return new double[][]{};
    }

    /**
     * Returns the Gaussian quadrature length of the parametric from a start to end t parameter
     * @param start t parameter to start length calculation
     * @param end t parameter to end length calculation
     * @param steps number of steps (degree) of quadrature
     * @return the Gaussian quadrature length of the parametric from a start to end t parameter
     */
    public double getGaussianQuadratureLength(double start, double end, int steps) {
        double[][] coefficients = getCoefficients(steps);

        //we are trying to find integral of sqrt(x'(t)^2 + y'(t)^2) from start to end

        //integral bound transformation from [0,1] to [-1, 1]
        double half = (end - start) / 2.0;
        double avg = (start + end) / 2.0;
        double length = 0;
        for (double[] coefficient : coefficients) {
            //sqrt(x'(t)^2 + y'(t)^2)
            length += getDerivative(avg + half * coefficient[1], 1).magnitude() * coefficient[0];
        }
        return length * half;
    }

    /**
     * Returns the Gaussian quadrature length from t=0 to the end t parameter
     * @param end t parameter to end length calculation
     * @param steps number of steps (degree) of quadrature
     * @return the Gaussian quadrature length from t=0 to the end t parameter
     */
    public double getGaussianQuadratureLength(double end, int steps) {
        return getGaussianQuadratureLength(0, end, steps);
    }

    /**
     * Returns the Gaussian quadrature length from t=0 to t=1
     * @param steps number of steps (degree) of quadrature
     * @return the Gaussian quadrature length from t=0 to t=1
     */
    public double getGaussianQuadratureLength(int steps) {
        return getGaussianQuadratureLength(1.0, steps);
    }

    /**
     * Returns the magnitude of the radial acceleration given a curvature and velocity
     * @param curvature the curvature
     * @param velocityMagnitude
     * @return the magnitude of the radial acceleration given a curvature and velocity
     */
    public double getAccelerationMagnitudeFromCurvature(double curvature, double velocityMagnitude) {
        //a_rad = |V|^2 / R, curvature = 1/R
        return curvature * velocityMagnitude * velocityMagnitude;
    }

    /**
     * Returns the curvature of the parametric at the given t
     * @param t t parameter to get curvature of
     * @return the curvature of the parametric at the given t
     */
    public double getCurvature(double t) {
        Point2D d1 = getDerivative(t, 1);
        Point2D d2 = getDerivative(t, 2);

        //https://en.wikipedia.org/wiki/Curvature#In_terms_of_a_general_parametrization
        return (d1.getX() * d2.getY() - d2.getX() * d1.getY()) / Math.pow(d1.getX()*d1.getX() + d1.getY()*d1.getY(), 1.5);
    }

    /**
     * Returns the brute force length of the parametric from t=start to t=end
     * @param start t parameter to start length calculation
     * @param end t parameter to end length calculation
     * @param steps number of steps for length calculations
     * @return the brute force length of the parametric from t=start to t=end
     */
    public double getRawLength(double start, double end, double steps) {
        double stepSize = (end - start) / steps;
        double length = 0;

        //number of points = number of steps
        for(double i = start; i <= end; i += stepSize) {
            //manually calculate distance between consecutive points
            length += getPoint(i).distance(getPoint(i+stepSize));
        }

        return length;
    }

    /**
     * Returns the closest associated t value on the spline from a {@link Point2D} using Newton's method on the distance function
     * @param point the {@link Point2D} that to get closest point from
     * @param steps the number of steps to start Newton's method from
     * @param iterations the number of iterations to run Newton's method on a single step
     * @return the closest associated t value on the spline from a {@link Point2D} using Newton's method on the distance function
     */
    public double findClosestPointOnSpline(Point2D point, int steps, int iterations) {

        Vector2D cur_min = new Vector2D(Double.POSITIVE_INFINITY, 0);

        //the steps to start Newton's method from
        for(double i = 0; i <= 1; i += 1./steps) {
            double cur_t = i;
            //get first and secondary derivatives of the distance function at that point
            Vector2D derivs = getDerivsAtT(cur_t, point);

            //amount to adjust according to Newton's method
            //https://en.wikipedia.org/wiki/Newton%27s_method
            //using first and second derivatives because we want min of distance function (zero of its derivative)
            double dt = derivs.getX() / derivs.getY();

            int counter = 0;

            //run for certain number of iterations
            while(counter < iterations) {

                //adjust based on Newton's method, get new derivatives
                cur_t -= dt;
                derivs = getDerivsAtT(cur_t, point);
                dt = derivs.getX() / derivs.getY();
                counter++;
            }

            //if distance is less than previous min, update distance and t
            double cur_d = getDistanceAtT(cur_t, point);

            if(cur_d < cur_min.getX()) {
                cur_min = new Vector2D(cur_d, cur_t);
            }
        }

        //return t of minimum distance, clamped from 0 to 1
        return Math.min(1, Math.max(0, cur_min.getY()));

    }

    /**
     * Returns the first and second derivatives of the (squared) distance function from a {@link Point2D} to a t on the spline
     * @param t t value to get distance from on the spline
     * @param point {@link Point2D} point to get distance from
     * @return
     */
    public Vector2D getDerivsAtT(double t, Point2D point) {
        //D2 = (x1(t) - x2)^2 + (y1(t) - y2)^2
        //D2' = 2(x1(t) - x2) * (x1'(t)) + 2(y1(t) - y2) * (y1'(t))
        //D2'' = 2(x1'(t)^2 + (x1(t) - x2)*x1''(t)) + 2(y1'(t)^2 + (y1(t) - y2)*y1''(t))

        Point2D p = getPoint(t); //(x1(t), y1(t))
        Point2D d1 = getDerivative(t, 1); //(x1'(t), y1'(t))
        Point2D d2 = getDerivative(t, 2); //(x1''(t), y1''(t))

        double x_a = p.getX() - point.getX(); // (x1(t) - x2)
        double y_b = p.getY() - point.getY(); // (y1(t) - y2)

        return new Vector2D(
                2*(x_a*d1.getX() + y_b*d1.getY()), //D2'
                2*(d1.getX() * d1.getX() + x_a*d2.getX() + d1.getY() * d1.getY() + y_b * d2.getY()) //D2''
        );
    }

    /**
     * Returns the squared distance from the spline at t to a {@link Point2D}
     * @param t t parameter of the spline to get distance from
     * @param point {@link Point2D} to get distance to spline from
     * @return the distance from the spline at t to a {@link Point2D}
     */
    public double getDistanceAtT(double t, Point2D point) {
        Point2D p = getPoint(t);
        //D2 = (x1(t) - x2)^2 + (y1(t) - y2)^2
        return (p.getX() - point.getX())*(p.getX() - point.getX()) +
                (p.getY() - point.getY())*(p.getY() - point.getY());
    }

    /**
     * Returns the t parameter associated with a certain length from the beginning
     * @param length length to get the t parameter of
     * @return the t parameter associated with a certain length from the beginning
     */
    public double getTFromLength(double length) {
        //approximate t: desired length divided by total length
        double t = length / this.length;

        for(int i = 0; i < 5; i++) {
            //magnitude of the derivative
            double derivativeMagnitude = getDerivative(t, 1).magnitude();

            //Newton's method: length remaining length divided by derivative
            if(derivativeMagnitude > 0.0) {
                t -= (getGaussianQuadratureLength(t, 11) - length) / derivativeMagnitude;
                //Clamp to [0, 1]
                t = Math.min(1, Math.max(t, 0));
            }
        }

        return t;
    }

    /**
     * Returns a {@link Vector2D} with the maximum x and y coordinates on the path
     * @param steps number of points to sample
     * @return a {@link Vector2D} with the maximum x and y coordinates on the path
     */
    public Vector2D getAbsoluteMaxCoordinates(double steps) {
        Vector2D max = new Vector2D();

        double stepsize = 1/steps;
        for(double t = 0; t <= 1; t += stepsize) {
            Point2D point = getPoint(t);
            max.x = Math.max(max.x, Math.abs(point.getX()));
            max.y = Math.max(max.y, Math.abs(point.getY()));
        }

        return max;
    }

    /**
     * Return a new {@link Parametric} path to this parametric's setpoint from a position, velocity, and acceleration
     * @param newPos {@Pose2D} to start from
     * @param newVel {@Vector2D} velocity to start from
     * @param newAcc {@Vector2D} acceleration to start from
     * @return a new {@link Parametric} path to this parametric's setpoint from a position, velocity, and acceleration
     */
    public Parametric getNewPath(Pose2D newPos, Vector2D newVel, Vector2D newAcc) {
        if(this instanceof QuinticHermiteSpline) {
            //for quintic hermite splines
            return new QuinticHermiteSpline(newPos, ((QuinticHermiteSpline) this).getPose1(), newVel,
                    ((QuinticHermiteSpline) this).getVelocity1(), newAcc, ((QuinticHermiteSpline) this).getAcceleration1());
        }
        return new Parametric();
    }

}
