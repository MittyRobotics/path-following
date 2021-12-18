package splines;

import math.Angle;
import math.Point2D;
import math.Pose2D;
import path.DifferentialDriveState;
import path.Path;

import java.text.DecimalFormat;

public class Test {
    public static void main(String[] args) {
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose2D(-200 * Path.TO_METERS, -100 * Path.TO_METERS, 0.7),
                new Pose2D(100 * Path.TO_METERS, 50 * Path.TO_METERS, 0.5)
        );

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(4);
        df.setMinimumFractionDigits(4);

        for(double t = 0; t <= 1; t+=0.01) {
            Point2D point = spline.getPoint(t);
            System.out.print("(" + df.format(point.getX()) + ", " + df.format(point.getY()) + "), ");
        }
        System.out.println();
        Path path = new Path(spline, 80 * Path.TO_METERS, 80 * Path.TO_METERS, 50 * Path.TO_METERS, 20 * Path.TO_METERS, 30 * Path.TO_METERS, 0);

//        Pose2D robotPosition = new Pose2D(0, 0, 0);


        double trackwidth = 25 * 0.0254;
        double tracklength = 37 * 0.0254;

        PathVisualizer visualizer = new PathVisualizer(path, 10*Path.TO_METERS, 1*Path.TO_METERS, 3*Path.TO_METERS, 30, trackwidth, tracklength);

        visualizer.visualize();
    }
}
