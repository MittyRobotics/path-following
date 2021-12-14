package splines;

import math.*;
import path.DifferentialDriveState;
import path.Path;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.lang.reflect.InvocationTargetException;
import java.sql.Array;
import java.text.DecimalFormat;
import java.util.ArrayList;
import javax.swing.*;
import javax.swing.border.Border;

public class PathVisualizer {

    private Path path;
    private double LOOKAHEAD, THRESHOLD, dt, TRACKWIDTH, TRACKLENGTH, cur_time;
    private Pose2D robotPosition;

    private double METERS_TO_PIXELS;
    private int TITLE_HEIGHT, FRAME_HEIGHT, FRAME_WIDTH;
    private ArrayList<Pose2D> robotPoses = new ArrayList<>();
    private ArrayList<Vector2D> velocities = new ArrayList<>();
    private ArrayList<Double> angularVelocities = new ArrayList<>();
    private ArrayList<Double> curvatures = new ArrayList<>();
    private ArrayList<Double> linearVelocities = new ArrayList<>();

    private int cur_pos_index = 0;
    private JFrame frame;
    private JComponent component;
    private JButton runSim;

    boolean simulating = false;

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

        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));

        //draw grid
        g.setColor(Color.WHITE);
        g.fillRect(0, 0, FRAME_WIDTH+400, FRAME_HEIGHT);

        int counter = 1;
        for(double i = METERS_TO_PIXELS * 12 * Path.TO_METERS; i <= FRAME_WIDTH/2.; i += METERS_TO_PIXELS * 12 * Path.TO_METERS) {
            g.setColor(Color.LIGHT_GRAY);
            g.fillRect((int) (FRAME_WIDTH/2 + i - 0.5), 0, 1, FRAME_HEIGHT);
            g.fillRect((int) (FRAME_WIDTH/2 - i - 0.5), 0, 1, FRAME_HEIGHT);

            g.setColor(Color.DARK_GRAY);
            g.drawString(" " + (counter/10 == 0 ? " " : "")  + counter, (int) (FRAME_WIDTH/2 + i - 15), FRAME_HEIGHT/2+15);
            g.drawString((counter/10 == 0 ? " " : "") + "-" + counter, (int) (FRAME_WIDTH/2 - i - 15), FRAME_HEIGHT/2+15);

            counter++;
        }

        counter = 1;
        for(double i = METERS_TO_PIXELS * 12 * Path.TO_METERS; i <= FRAME_HEIGHT/2.; i += METERS_TO_PIXELS * 12 * Path.TO_METERS) {
            g.setColor(Color.LIGHT_GRAY);
            g.fillRect(0, (int) (FRAME_HEIGHT/2 + i - 0.5), FRAME_WIDTH, 1);
            g.fillRect(0, (int) (FRAME_HEIGHT/2 - i - 0.5), FRAME_WIDTH, 1);

            g.setColor(Color.DARK_GRAY);
            g.drawString(" " + (counter/10 == 0 ? " " : "") + counter, FRAME_WIDTH/2-25, (int) (FRAME_HEIGHT/2 + i + 5));
            g.drawString((counter/10 == 0 ? " " : "") + "-" + counter, FRAME_WIDTH/2-25, (int) (FRAME_HEIGHT/2 - i + 5));
            counter++;
        }

        g.setColor(Color.BLACK);
        g.fillRect(FRAME_WIDTH/2-1, 0, 2, FRAME_HEIGHT);
        g.fillRect(0, FRAME_HEIGHT/2-1, FRAME_WIDTH, 2);

//        g.drawString("1 ft", (int) (FRAME_WIDTH/2 + METERS_TO_PIXELS * 12 * Path.TO_METERS) - 10, FRAME_HEIGHT/2 + 20);


        //draw robot

        Angle angle = robotPoses.get(cur_pos_index).getAngle();
        Angle angle1 = new Angle(Math.PI/2 + angle.getAngle());

        Point2D pos = new Point2D(robotPoses.get(cur_pos_index).getPosition().x, robotPoses.get(cur_pos_index).getPosition().y);

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

        g.setStroke(new BasicStroke(5));

        for(double i = 0; i < 1; i += 0.001) {
            Point2D start = path.getParametric().getPoint(i);
            Point2D end = path.getParametric().getPoint(i+0.001);

            g.drawLine(convertXToPixels(start.x), convertYToPixels(start.y),
                    convertXToPixels(end.x), convertYToPixels(end.y));
        }

        g.setColor(new Color(150, 0, 0));
        Point2D start = path.getParametric().getPoint(0);
        Point2D end = path.getParametric().getPoint(1);

        g.fillOval(convertXToPixels(start.x)-5, convertYToPixels(start.y)-5, 10, 10);
        g.fillOval(convertXToPixels(end.x)-5, convertYToPixels(end.y)-5, 10, 10);

        //draw path
        g.setColor(Color.CYAN);

        g.setStroke(new BasicStroke(3));

        g.fillOval(convertXToPixels(pos.x)-5, convertYToPixels(pos.y)-5, 10, 10);

        for(int i = 0; i < cur_pos_index; i++) {
            Pose2D pose1 = robotPoses.get(i);
            Pose2D pose2 = robotPoses.get(i+1);

            g.drawLine(convertXToPixels(pose1.getPosition().x), convertYToPixels(pose1.getPosition().y),convertXToPixels(pose2.getPosition().x), convertYToPixels(pose2.getPosition().y));
        }



        //draw text

        g.setColor(Color.BLACK);
        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 24));
        g.drawString("Path Following Simulator", FRAME_WIDTH + 30, 80);

        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        df.setMinimumFractionDigits(3);

        g.drawString("Velocity: " + df.format(linearVelocities.get(cur_pos_index)*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 330);
        g.drawString("Left Velocity: " + df.format(velocities.get(cur_pos_index).getX()*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 360);
        g.drawString("Right Velocity: " + df.format(velocities.get(cur_pos_index).getY()*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 390);

        Vector2D acc = new Vector2D();
        double lacc = 0;
        if(cur_pos_index != 0 && cur_pos_index != velocities.size()-1) {
            acc = new Vector2D((velocities.get(cur_pos_index).x - velocities.get(cur_pos_index-1).x)/dt,
                    (velocities.get(cur_pos_index).y - velocities.get(cur_pos_index-1).y)/dt);
            lacc = (linearVelocities.get(cur_pos_index) - linearVelocities.get(cur_pos_index-1))/dt;
        }

        g.drawString("Acceleration: " + df.format(lacc*Path.TO_INCHES) + " in/s^2", FRAME_WIDTH+30, 440);
        g.drawString("Left Acceleration: " + df.format(acc.x*Path.TO_INCHES) + " in/s^2", FRAME_WIDTH+30, 470);
        g.drawString("Right Acceleration: " + df.format(acc.y*Path.TO_INCHES) + " in/s^2", FRAME_WIDTH+30, 500);

        g.drawString("Angular Velocity: " + df.format(angularVelocities.get(cur_pos_index)*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 550);
        g.drawString("Curvature: " + df.format(curvatures.get(cur_pos_index)*Path.TO_METERS) + " in^-1", FRAME_WIDTH+30, 580);

        g.drawString("Position: " + "(" + df.format(pos.x*Path.TO_INCHES) + " in, " + df.format(pos.y*Path.TO_INCHES) + " in)", FRAME_WIDTH+30, 630);
        g.drawString("Angle: " + df.format(angle.getAngle() * 180 / Math.PI) + " °", FRAME_WIDTH+30, 660);
        g.drawString("Endpoint: " + "(" + df.format(end.x*Path.TO_INCHES) + " in, " + df.format(end.y*Path.TO_INCHES) + " in)", FRAME_WIDTH+30, 710);
        double endAngle = path.getParametric().getPose(1).getAngleRadians();
        g.drawString("End Angle: " + df.format(endAngle * 180 / Math.PI) + " °", FRAME_WIDTH+30, 740);

    }

    public void runSimulation() {
        if(!simulating) {
            if(runSim.getText().equals("RUN SIM")) {cur_pos_index = 0;}
            simulating = true;
            runSim.setText("STOP");
        } else {
            simulating = false;
            runSim.setText("CONTINUE");
        }
    }

    public boolean visualize(Pose2D startPosition, double END_TIME) {

        frame = new JFrame();

        TITLE_HEIGHT = 28;
        FRAME_WIDTH = 800;
        FRAME_HEIGHT = 800;

        Vector2D maxPoint = path.getParametric().getAbsoluteMaxCoordinates(1000);
        double maxMeters = Math.max(maxPoint.getX(), maxPoint.getY()) * 7./5.;

        METERS_TO_PIXELS = 400 / maxMeters;

        frame.setSize(FRAME_WIDTH+400, FRAME_HEIGHT + TITLE_HEIGHT);
        frame.setResizable(false);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


        component = new JPanel() {
            public void paintComponent(Graphics g) {
                Graphics2D g2 = (Graphics2D) g;
                g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
                draw(g2);
            }
        };

        runSim = new JButton("RUN SIM");
        runSim.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 18));
        runSim.setBounds(FRAME_WIDTH+125, 200, 150, 50);
        runSim.addActionListener(e -> runSimulation());

        component.add(runSim);
        component.setLayout(null);
        frame.add(component);

//        frame.setLayout(null);
        frame.setVisible(true);



        cur_time = 0;
        robotPosition = startPosition;

        //http://rossum.sourceforge.net/papers/DiffSteer/


        robotPoses.clear();

        curvatures.add(path.getCurvature(robotPosition));
        robotPoses.add(robotPosition);
        velocities.add(new Vector2D(0, 0));
        angularVelocities.add(0.);
        linearVelocities.add(0.);

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

            curvatures.add(path.getCurvature(robotPosition));

            robotPosition = new Pose2D(new_x, new_y, newAngle);

            robotPoses.add(robotPosition);
            velocities.add(new Vector2D(dds.getLeftVelocity(), dds.getRightVelocity()));
            angularVelocities.add(dds.getAngularVelocity());
            linearVelocities.add(dds.getLinearVelocity());

            if(path.isFinished(robotPosition, THRESHOLD)) {

                curvatures.add(path.getCurvature(robotPosition));

                robotPoses.add(robotPosition);
                velocities.add(new Vector2D(0, 0));
                angularVelocities.add(0.);
                linearVelocities.add(0.);

                break;
            }

        }

        while(true) {
            if(simulating) {
                component.updateUI();

                for (int i = cur_pos_index; i < robotPoses.size(); i++) {
                    if(simulating) {
                        try {
                            Thread.sleep((long) (dt * 1000));

                            cur_pos_index = i;
                            component.updateUI();

                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    } else {
                        break;
                    }
                }

                if(simulating) {
                    runSim.setText("RUN SIM");
                    simulating = false;
                }
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

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
