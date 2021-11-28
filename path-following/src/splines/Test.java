package splines;

import math.Pose2D;

import java.text.DecimalFormat;

public class Test {
    public static void main(String[] args) {
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose2D(0, 0, 0),
                new Pose2D(0, 50, 0)
        );

        DecimalFormat df = new DecimalFormat("#");
        df.setMaximumFractionDigits(5);

        for(double i = 0; i <= 1.0; i+=0.01) {
            System.out.print("(" + df.format(spline.getPoint(i).getX()) + ", " + df.format(spline.getPoint(i).getY()) + "),");
        }
        System.out.println();

        System.out.println(spline.getRawLength(0., 1., 10000));
        System.out.println(spline.getGaussianQuadratureLength(0., 1.));
    }
}
