package splines;

import math.Angle;
import math.Point2D;
import math.Pose2D;
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
        Path path = new Path(spline, 30, 30);

        Pose2D robotPosition = new Pose2D(0, 0, 0);


        double trackwidth = 0.635;

        while(!path.isFinished(robotPosition, 0.0254)) {
            DifferentialDriveState dds = path.update(robotPosition, 0.02, 0.25, trackwidth);
            double left = dds.getLeftVelocity() * 0.02;
            double right = dds.getRightVelocity() * 0.02;
            Angle angle = robotPosition.getAngle();
            double x = robotPosition.getPosition().getX();
            double y = robotPosition.getPosition().getY();
            double new_x, new_y, newAngle;

            x += (Math.random()-0.5)*0.01;
            y += (Math.random()-0.5)*0.01;

            if(Math.abs(left - right) < 1e-6) {
                new_x = x + left * angle.cos();
                new_y = y + right * angle.sin();
                newAngle = angle.getAngle();
            } else {
                double R = trackwidth * (left + right) / (2 * (right - left));
                double wd = (right - left) / trackwidth;

                new_x = x + R * Math.sin(angle.getAngle() + wd) - R * angle.sin();
                new_y = y - R * Math.cos(angle.getAngle() + wd) + R * angle.cos();
                newAngle = Math.toRadians(Math.toDegrees(angle.getAngle() + wd));
            }


            robotPosition = new Pose2D(new_x, new_y, newAngle);

            robotPosition.getPosition().print();
            robotPosition.getAngle().print();
        }

        System.out.print("Endpoint: ");
        spline.getPoint(1.0).print();


    }
}
