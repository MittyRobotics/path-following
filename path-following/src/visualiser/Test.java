package visualiser;

import math.Point2D;
import math.Pose2D;
import math.Vector2D;
import path.Path;
import path.RamsetePath;
import splines.QuinticHermiteSpline;

import java.text.DecimalFormat;

public class Test {
    public static void main(String[] args) {
        QuinticHermiteSpline spline = new QuinticHermiteSpline(
                new Pose2D(-30 * Path.TO_METERS, -30 * Path.TO_METERS, 20 * Math.PI/180),
                new Pose2D(10 * Path.TO_METERS, 20 * Path.TO_METERS, 0 * Math.PI/180)
        );

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(4);
        df.setMinimumFractionDigits(4);

//        for(double t = 0; t <= 1; t+=0.01) {
//            Point2D point = spline.getPoint(t);
//            System.out.print("(" + df.format(point.getX()) + ", " + df.format(point.getY()) + "), ");
//        }
//        System.out.println();

        Path path = new Path(spline, 80 * Path.TO_METERS, 80 * Path.TO_METERS, 50 * Path.TO_METERS, 40 * Path.TO_METERS, 0 * Path.TO_METERS, 0 * Path.TO_METERS);


        double trackwidth = 25 * 0.0254;
        double tracklength = 37 * 0.0254;

        PathVisualizer visualizer = new PathVisualizer(path, 2*Path.TO_METERS, 0.2*Path.TO_METERS, 3*Path.TO_METERS, 30, trackwidth, tracklength);

        visualizer.run();

//        RamsetePath path = new RamsetePath(spline, 80 * Path.TO_METERS, 80 * Path.TO_METERS, 50 * Path.TO_METERS, 20 * Path.TO_METERS, 0 * Path.TO_METERS, 0 * Path.TO_METERS);
//
//        double trackwidth = 25 * 0.0254;
//        double tracklength = 37 * 0.0254;
//
//        double b = 2;
//        double Z = 0.2;
//
//        RamsetePathVisualizer visualizer = new RamsetePathVisualizer(path, 1*Path.TO_METERS, 3*Path.TO_METERS, 30, trackwidth, tracklength, b, Z, 0.02);
//
//        visualizer.run();
    }
}
