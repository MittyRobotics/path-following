package splines;

import math.*;
import path.DifferentialDriveState;
import path.Path;

import java.awt.*;
import java.awt.event.ActionListener;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import javax.swing.*;

public class PathVisualizer {

    private Path path;
    private double LOOKAHEAD, THRESHOLD, dt, TRACKWIDTH, TRACKLENGTH, cur_time;
    private Pose2D robotPosition;

    private double METERS_TO_PIXELS;
    private int TITLE_HEIGHT, FRAME_HEIGHT, FRAME_WIDTH;
    private ArrayList<Pose2D> robotPoses = new ArrayList<>();

    public PathVisualizer(Path path, double LOOKAHEAD, double THRESHOLD, double TRACKWIDTH, double TRACKLENGTH, double dt) {
        this.path = path;
        this.LOOKAHEAD = LOOKAHEAD;
        this.THRESHOLD = THRESHOLD;
        this.TRACKWIDTH = TRACKWIDTH;
        this.TRACKLENGTH = TRACKLENGTH;

        this.dt = dt;
    }

    public PathVisualizer(Path path, double LOOKAHEAD, double THRESHOLD, double TRACKWIDTH, double TRACKLENGTH) {
        this(path, LOOKAHEAD, THRESHOLD, TRACKWIDTH, TRACKLENGTH, 0.02);
    }

    public void updatePath(Path path, double LOOKAHEAD, double THRESHOLD) {
        this.path = path;
        this.LOOKAHEAD = LOOKAHEAD;
        this.THRESHOLD = THRESHOLD;
    }

    public void draw(Graphics2D g) {
        //draw grid
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, FRAME_WIDTH, FRAME_HEIGHT);

        g.setColor(Color.LIGHT_GRAY);
        for(double i = METERS_TO_PIXELS * 12 * Path.TO_METERS; i <= FRAME_WIDTH; i += METERS_TO_PIXELS * 12 * Path.TO_METERS) {
            g.fillRect((int) (FRAME_WIDTH/2 + i - 0.5), 0, 1, FRAME_HEIGHT);
            g.fillRect((int) (FRAME_WIDTH/2 - i - 0.5), 0, 1, FRAME_HEIGHT);
        }

        for(double i = METERS_TO_PIXELS * 12 * Path.TO_METERS; i <= FRAME_HEIGHT; i += METERS_TO_PIXELS * 12 * Path.TO_METERS) {
            g.fillRect(0, (int) (FRAME_HEIGHT/2 + i - 0.5), FRAME_WIDTH, 1);
            g.fillRect(0, (int) (FRAME_HEIGHT/2 - i - 0.5), FRAME_WIDTH, 1);
        }

        g.setColor(Color.BLACK);
        g.fillRect(FRAME_WIDTH/2-1, 0, 2, FRAME_HEIGHT);
        g.fillRect(0, FRAME_HEIGHT/2-1, FRAME_WIDTH, 2);


        //draw robot

        Angle angle = robotPosition.getAngle();
        Angle angle1 = new Angle(Math.PI/2 + robotPosition.getAngle().getAngle());

        Point2D pos = new Point2D(robotPosition.getPosition().x, robotPosition.getPosition().y);

        Point2D front = new Point2D(pos.x + TRACKLENGTH/2*angle.cos(), pos.y + TRACKLENGTH/2*angle.sin());
        Point2D back = new Point2D(pos.x - TRACKLENGTH/2*angle.cos(), pos.y - TRACKLENGTH/2*angle.sin());

        Point2D frontLeft = new Point2D(front.x + TRACKWIDTH/2*angle1.cos(), front.y + TRACKWIDTH/2*angle1.sin());
        Point2D frontRight = new Point2D(front.x - TRACKWIDTH/2*angle1.cos(), front.y - TRACKWIDTH/2*angle1.sin());

        Point2D backLeft = new Point2D(back.x + TRACKWIDTH/2*angle1.cos(), back.y + TRACKWIDTH/2*angle1.sin());
        Point2D backRight = new Point2D(back.x - TRACKWIDTH/2*angle1.cos(), back.y - TRACKWIDTH/2*angle1.sin());

        g.setColor(Color.BLUE);

        int[] x_comp = new int[]{convertXToPixels(frontLeft.x), convertXToPixels(frontRight.x), convertXToPixels(backRight.x), convertXToPixels(backLeft.x)};
        int[] y_comp = new int[]{convertYToPixels(frontLeft.y), convertYToPixels(frontRight.y), convertYToPixels(backRight.y), convertYToPixels(backLeft.y)};
        g.fillPolygon(x_comp, y_comp, 4);


        //draw spline
        g.setColor(Color.RED);

        g.setStroke(new BasicStroke(3));

        for(double i = 0; i < 1; i += 0.001) {
            Point2D start = path.getParametric().getPoint(i);
            Point2D end = path.getParametric().getPoint(i+0.001);

            g.drawLine(convertXToPixels(start.x), convertYToPixels(start.y),
                    convertXToPixels(end.x), convertYToPixels(end.y));
        }

        g.setColor(new Color(100, 0, 0));
        Point2D start = path.getParametric().getPoint(0);
        Point2D end = path.getParametric().getPoint(1);

        g.fillOval(convertXToPixels(start.x)-3, convertYToPixels(start.y)-3, 6, 6);
        g.fillOval(convertXToPixels(end.x)-3, convertYToPixels(end.y)-3, 6, 6);

        //draw path
        g.setColor(Color.CYAN);

        g.setStroke(new BasicStroke(1));

        g.fillOval(convertXToPixels(pos.x)-3, convertYToPixels(pos.y)-3, 6, 6);

        for(int i = 0; i < robotPoses.size() - 1; i++) {
            Pose2D pose1 = robotPoses.get(i);
            Pose2D pose2 = robotPoses.get(i+1);

            g.drawLine(convertXToPixels(pose1.getPosition().x), convertYToPixels(pose1.getPosition().y),convertXToPixels(pose2.getPosition().x), convertYToPixels(pose2.getPosition().y));
        }
    }

    public boolean visualize(Pose2D startPosition, double END_TIME) {

        JFrame frame = new JFrame();

        TITLE_HEIGHT = 28;
        FRAME_WIDTH = 800;
        FRAME_HEIGHT = 800;

        Vector2D maxPoint = path.getParametric().getAbsoluteMaxCoordinates(1000);
        double maxMeters = Math.max(maxPoint.getX(), maxPoint.getY()) * 7./5.;

        METERS_TO_PIXELS = 400 / maxMeters;

        frame.setSize(FRAME_WIDTH, FRAME_HEIGHT + TITLE_HEIGHT);
        frame.setResizable(false);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


        JPanel component = new JPanel() {
            public void paintComponent(Graphics g) {
                Graphics2D g2 = (Graphics2D) g;
                draw(g2);
            }
        };

        frame.add(component);
        frame.setVisible(true);


        cur_time = 0;
        robotPosition = startPosition;

        //http://rossum.sourceforge.net/papers/DiffSteer/


        robotPoses.clear();

        while(cur_time < END_TIME) {

            DifferentialDriveState dds = path.update(robotPosition, dt, LOOKAHEAD, TRACKWIDTH);
            double left = dds.getLeftVelocity() * dt;
            double right = dds.getRightVelocity() * dt;
            Angle angle = robotPosition.getAngle();
            double x = robotPosition.getPosition().getX();
            double y = robotPosition.getPosition().getY();
            double new_x, new_y, newAngle;

            if(Math.abs(left - right) < 1e-6) {
                new_x = x + left * angle.cos();
                new_y = y + right * angle.sin();
                newAngle = angle.getAngle();
            } else {
                double turnRadius = TRACKWIDTH * (left + right) / (2 * (right - left));
                newAngle = Math.toRadians(Math.toDegrees(angle.getAngle() + (right - left) / TRACKWIDTH));

                new_x = x + turnRadius * (Math.sin(newAngle) - angle.sin());
                new_y = y - turnRadius * (Math.cos(newAngle) - angle.cos());
            }

            cur_time += dt;

            robotPosition = new Pose2D(new_x, new_y, newAngle);

            robotPoses.add(robotPosition);


            if(path.isFinished(robotPosition, THRESHOLD)) {
                return true;
            }

            try {
                Thread.sleep((long) (dt*1000));

                component.updateUI();

            } catch (Exception e) {
                e.printStackTrace();
            }

        }

        return false;
    }

    public boolean visualize() {
        return this.visualize(path.getParametric().getPose(0), 180);
    }

    public int convertXToPixels(double x) {
        return FRAME_WIDTH/2 + (int) (x * METERS_TO_PIXELS);
    }

    public int convertYToPixels(double y) {
        return FRAME_HEIGHT/2 - (int) (y * METERS_TO_PIXELS);
    }

}
