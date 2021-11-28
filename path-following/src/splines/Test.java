package splines;

import math.*;
import path.Path;

import java.text.DecimalFormat;

public class Test {
    public static void main(String[] args) {
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose2D(0, 0, 0),
                new Pose2D(0, 50, 0)
        );

        DecimalFormat df = new DecimalFormat("#");
        df.setMaximumFractionDigits(10);

        for(double i = 0; i <= 1.0; i+=0.01) {
            System.out.print("(" + df.format(spline.getPoint(i).getX()) + ", " + df.format(spline.getPoint(i).getY()) + "),");
        }
        System.out.println();

        /*double actualLength = spline.getRawLength(0., 1., 100000);
        for(int i = 2; i <= 20; i++) {
            System.out.println("n=" + i + " | val=" + spline.getGaussianQuadratureLength(0., 1., i)
            + " | error=" + (actualLength - spline.getGaussianQuadratureLength(0., 1., i)) + " | percenterror = " +
                    df.format((actualLength - spline.getGaussianQuadratureLength(0., 1., i))/actualLength));
        }*/

        /*Pose2D pose = new Pose2D(new Point2D(0, 0), new Angle(0));
        Point2D other = new Point2D(2, -1);

        Circle circle = new Circle();
        circle.fromTangentAndPoint(pose, other);

        circle.print();*/

//        long time = System.currentTimeMillis();

        Path path = new Path(spline, 0, 0);

        Point2D point = new Point2D(10, 30);

        for(double i = 0; i <= 1.0; i+=0.01) {
            System.out.print("(" + i + ", " + df.format(path.getDerivsAtT(i, point).getX()) + "),");
        }
        System.out.println();

        for(double i = 0; i <= 1.0; i+=0.01) {
            System.out.print("(" + i + ", " + df.format(path.getDerivsAtT(i, point).getY()) + "),");
        }
        System.out.println();


        double result = path.findClosestPointOnSpline(point, 0.001, 10, 10);

//        System.out.println(System.currentTimeMillis() - time);

        spline.getPoint(result).print();

    }
}
