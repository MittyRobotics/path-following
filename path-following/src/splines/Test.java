package splines;

import math.Pose2D;

import java.text.DecimalFormat;

public class Test {
    public static void main(String[] args) {
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose2D(0, 0, 0),
                new Pose2D(100, 50, -1)
        );

        DecimalFormat df = new DecimalFormat("#");
        df.setMaximumFractionDigits(10);

        /*for(double i = 0; i <= 1.0; i+=0.01) {
            System.out.print("(" + df.format(spline.getPoint(i).getX()) + ", " + df.format(spline.getPoint(i).getY()) + "),");
        }
        System.out.println();*/

        /*double actualLength = spline.getRawLength(0., 1., 100000);
        for(int i = 2; i <= 20; i++) {
            System.out.println("n=" + i + " | val=" + spline.getGaussianQuadratureLength(0., 1., i)
            + " | error=" + (actualLength - spline.getGaussianQuadratureLength(0., 1., i)) + " | percenterror = " +
                    df.format((actualLength - spline.getGaussianQuadratureLength(0., 1., i))/actualLength));
        }*/
    }
}
