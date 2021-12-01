package splines;

import math.*;
import path.DifferentialDriveState;
import path.Path;

import java.text.DecimalFormat;

public class Test {
    public static void main(String[] args) {
//        QuinticHermiteSpline spline = new QuinticHermiteSpline(
//                new Pose2D(0, 0, 0),
//                new Pose2D(0, 50, 0)
//        );
//
//        DecimalFormat df = new DecimalFormat("#");
//        df.setMaximumFractionDigits(10);
//
//        for(double i = 0; i <= 1.0; i+=0.01) {
//            System.out.print("(" + df.format(spline.getPoint(i).getX()) + ", " + df.format(spline.getPoint(i).getY()) + "),");
//        }
//        System.out.println();

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

        /*Path path = new Path(spline, 30, 30);

        DifferentialDriveState dds = path.update(new Pose2D(new Point2D(1, 0), new Angle(0.1)), 0.02, 0.05,20);
        System.out.println(dds.getLeftVelocity() + " " + dds.getRightVelocity());*/


        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose2D(0, 0, 0),
                new Pose2D(2.54, 1.72, 0)
        );

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(4);
        df.setMinimumFractionDigits(4);

        for(double t = 0; t <= 1; t+=0.01) {
            Point2D point = spline.getPoint(t);
            System.out.print("(" + df.format(point.getX()) + ", " + df.format(point.getY()) + "), ");
        }
        System.out.println();
        Path path = new Path(spline, 50, 50);

        path.update(new Pose2D(2.4787, 1.7199, 0.5), 0.2, 0.254, 25);
//        DifferentialDriveState dds = path.update(new Pose2D(new Point2D(1, 0), new Angle(0.1)), 0.02, 0.05,20);
    }
}
