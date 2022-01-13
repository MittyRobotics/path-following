package visualiser;

import math.*;
import path.DifferentialDriveState;
import path.Path;
import path.RamseteController;
import path.RamsetePath;
import splines.Parametric;
import splines.QuinticHermiteSpline;

import java.awt.*;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import javax.swing.*;

public class RamsetePathVisualizer {

    private RamsetePath path;
    private double END_THRESHOLD;
    private double ADJUST_THRESHOLD;
    private final double dt;
    private final double TRACKWIDTH;
    private final double TRACKLENGTH;
    private double b;
    private double Z;
    private double cur_time;
    private double END_TIME;
    private Pose2D robotPosition, startPosition;

    private double METERS_TO_PIXELS;
    private int TITLE_HEIGHT;
    private int FRAME_HEIGHT;
    private int FRAME_WIDTH;
    private int ADJUST_FRAME_HEIGHT;
    private int ADJUST_FRAME_WIDTH;
    private int NEWTONS_STEPS;
    private final ArrayList<Pose2D> robotPoses = new ArrayList<>();
    private final ArrayList<Vector2D> velocities = new ArrayList<>();
    private final ArrayList<Double> angularVelocities = new ArrayList<>();
    private final ArrayList<Double> curvatures = new ArrayList<>();
    private final ArrayList<Double> linearVelocities = new ArrayList<>();
    private final ArrayList<Parametric> parametrics = new ArrayList<>();


    private final ArrayList<Double> vel = new ArrayList<>();
    private final ArrayList<Double> angVel = new ArrayList<>();
    private final ArrayList<Double> rvel = new ArrayList<>();
    private final ArrayList<Double> rangVel = new ArrayList<>();
    private final ArrayList<Double> ex = new ArrayList<>();
    private final ArrayList<Double> ey = new ArrayList<>();
    private final ArrayList<Double> et = new ArrayList<>();
    private final ArrayList<Double> k = new ArrayList<>();
    private final ArrayList<Double> t = new ArrayList<>();
    private final ArrayList<Double> td = new ArrayList<>();

    private int cur_pos_index = 0;
    private JFrame frame, adjustFrame;
    private JComponent component, adjustComponent;
    private JButton runSimButton, adjustButton, updateButton, exportButton;
    private JSlider timeSlider;
    private ScheduledExecutorService executorService;
    private JTextField[] rightFields, leftFields;
    private final String[] rightLabels = {"Max Acceleration", "Max Deceleration", "Max Velocity", "Max Angular Vel.", "Start Velocity", "End Velocity", "Start Position X", "Start Position Y", "Start Angle", "b", "Z", "End Threshold", "Adjust Threshold", "Newton's Steps"};
    private final String[] leftLabels = {"Pose 0 X", "Pose 0 Y", "Pose 0 Angle", "Pose 1 X", "Pose 1 Y", "Pose 1 Angle", "Velocity 0 X", "Velocity 0 Y", "Velocity 1 X", "Velocity 1 Y", "Acceleration 0 X", "Acceleration 0 Y", "Acceleration 1 X", "Acceleration 1 Y"};

    boolean simulating = false;

    public RamsetePathVisualizer(RamsetePath path, double END_THRESHOLD, double ADJUST_THRESHOLD, int NEWTONS_STEPS, double TRACKWIDTH, double TRACKLENGTH, double b, double Z, double dt) {
        this.path = path;
        this.END_THRESHOLD = END_THRESHOLD;
        this.ADJUST_THRESHOLD = ADJUST_THRESHOLD;
        this.NEWTONS_STEPS = NEWTONS_STEPS;
        this.TRACKWIDTH = TRACKWIDTH;
        this.TRACKLENGTH = TRACKLENGTH;
        this.b = b;
        this.Z = Z;

        this.dt = dt;
    }

    public void draw(Graphics2D g) {

        if(timeSlider != null) {
            timeSlider.setValue(cur_pos_index);
            if (cur_pos_index == velocities.size() - 1) {
                runSimButton.setText("RUN SIM");
            }
        }

        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));

        //draw grid
        g.setColor(new Color(230, 230, 230));
        g.fillRect(0, 0, FRAME_WIDTH+400, FRAME_HEIGHT);

        int counter = 1;
        for(double i = METERS_TO_PIXELS * 12 * Path.TO_METERS; i <= FRAME_WIDTH/2.; i += METERS_TO_PIXELS * 12 * Path.TO_METERS) {
            g.setColor(Color.LIGHT_GRAY);
            g.fillRect((int) (FRAME_WIDTH/2 + i - 0.5), 0, 1, FRAME_HEIGHT);
            g.fillRect((int) (FRAME_WIDTH/2 - i - 0.5), 0, 1, FRAME_HEIGHT);

            if(counter % 5 == 0) {
                g.setColor(Color.DARK_GRAY);
                g.drawString(" " + (counter / 10 == 0 ? " " : "") + counter, (int) (FRAME_WIDTH / 2 + i - 15), FRAME_HEIGHT / 2 + 15);
                g.drawString((counter / 10 == 0 ? " " : "") + "-" + counter, (int) (FRAME_WIDTH / 2 - i - 15), FRAME_HEIGHT / 2 + 15);
            }

            counter++;
        }

        counter = 1;
        for(double i = METERS_TO_PIXELS * 12 * Path.TO_METERS; i <= FRAME_HEIGHT/2.; i += METERS_TO_PIXELS * 12 * Path.TO_METERS) {
            g.setColor(Color.LIGHT_GRAY);
            g.fillRect(0, (int) (FRAME_HEIGHT/2 + i - 0.5), FRAME_WIDTH, 1);
            g.fillRect(0, (int) (FRAME_HEIGHT/2 - i - 0.5), FRAME_WIDTH, 1);

            if(counter % 5 == 0) {
                g.setColor(Color.DARK_GRAY);
                g.drawString(" " + (counter / 10 == 0 ? " " : "") + counter, FRAME_WIDTH / 2 - 25, (int) (FRAME_HEIGHT / 2 - i + 5));
                g.drawString((counter / 10 == 0 ? " " : "") + "-" + counter, FRAME_WIDTH / 2 - 25, (int) (FRAME_HEIGHT / 2 + i + 5));
            }

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
            Point2D start = parametrics.get(cur_pos_index).getPoint(i);
            Point2D end = parametrics.get(cur_pos_index).getPoint(i+0.001);

            g.drawLine(convertXToPixels(start.x), convertYToPixels(start.y),
                    convertXToPixels(end.x), convertYToPixels(end.y));
        }

        g.setColor(new Color(150, 0, 0));
        Point2D start = parametrics.get(cur_pos_index).getPoint(0);
        Point2D end = parametrics.get(cur_pos_index).getPoint(1);

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
        g.drawString("Path Following Simulator", FRAME_WIDTH + 30, 70);

        g.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));

        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(3);
        df.setMinimumFractionDigits(3);

        g.drawString("Velocity: " + df.format(linearVelocities.get(cur_pos_index)*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 180);
        g.drawString("Left Velocity: " + df.format(velocities.get(cur_pos_index).getX()*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 210);
        g.drawString("Right Velocity: " + df.format(velocities.get(cur_pos_index).getY()*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 240);

        Vector2D acc = new Vector2D();
        double lacc = 0;
        if(cur_pos_index != 0) {
            acc = new Vector2D((velocities.get(cur_pos_index).x - velocities.get(cur_pos_index-1).x)/dt,
                    (velocities.get(cur_pos_index).y - velocities.get(cur_pos_index-1).y)/dt);
            lacc = (linearVelocities.get(cur_pos_index) - linearVelocities.get(cur_pos_index-1))/dt;
        }

        g.drawString("Acceleration: " + df.format(lacc*Path.TO_INCHES) + " in/s^2", FRAME_WIDTH+30, 290);
        g.drawString("Left Acceleration: " + df.format(acc.x*Path.TO_INCHES) + " in/s^2", FRAME_WIDTH+30, 320);
        g.drawString("Right Acceleration: " + df.format(acc.y*Path.TO_INCHES) + " in/s^2", FRAME_WIDTH+30, 350);

        g.drawString("Angular Velocity: " + df.format(angularVelocities.get(cur_pos_index)*Path.TO_INCHES) + " in/s", FRAME_WIDTH+30, 400);
        g.drawString("Curvature: " + df.format(curvatures.get(cur_pos_index)*Path.TO_METERS) + " in^-1", FRAME_WIDTH+30, 430);

        g.drawString("Position: " + "(" + df.format(pos.x*Path.TO_INCHES) + " in, " + df.format(pos.y*Path.TO_INCHES) + " in)", FRAME_WIDTH+30, 480);
        g.drawString("Distance From Spline: " + df.format(path.distanceFromSpline(parametrics.get(cur_pos_index), robotPoses.get(cur_pos_index), NEWTONS_STEPS) * Path.TO_INCHES) + " in", FRAME_WIDTH+30, 510);
        g.drawString("Angle: " + df.format(angle.getAngle() * 180 / Math.PI) + " °", FRAME_WIDTH+30, 540);
        g.drawString("Endpoint: " + "(" + df.format(end.x*Path.TO_INCHES) + " in, " + df.format(end.y*Path.TO_INCHES) + " in)", FRAME_WIDTH+30, 570);
        double endAngle = path.getParametric().getPose(1).getAngleRadians();
        g.drawString("End Angle: " + df.format(endAngle * 180 / Math.PI) + " °", FRAME_WIDTH+30, 600);

//        g.drawString("Orig Vel: " + df.format(vel.get(cur_pos_index)), FRAME_WIDTH + 30, 180);
//        g.drawString("Orig Ang Vel: " + df.format(angVel.get(cur_pos_index)), FRAME_WIDTH + 30, 210);
//
//        g.drawString("Ramsete Vel: " + df.format(rvel.get(cur_pos_index)), FRAME_WIDTH + 30, 250);
//        g.drawString("Ramsete Ang Vel: " + df.format(rangVel.get(cur_pos_index)), FRAME_WIDTH + 30, 280);
//
//        g.drawString("ex: " + df.format(ex.get(cur_pos_index)), FRAME_WIDTH + 30, 320);
//        g.drawString("ey: " + df.format(ey.get(cur_pos_index)), FRAME_WIDTH + 30, 350);
//        g.drawString("et: " + df.format(et.get(cur_pos_index)), FRAME_WIDTH + 30, 380);
//        g.drawString("k: " + df.format(k.get(cur_pos_index)), FRAME_WIDTH + 30, 420);
//
//        g.drawString("t: " + df.format(t.get(cur_pos_index)), FRAME_WIDTH + 30, 460);
//        g.drawString("td: " + df.format(td.get(cur_pos_index)), FRAME_WIDTH + 30, 490);
//

        g.drawString("Time Elapsed: " + df.format(cur_pos_index * 0.02) + " s", FRAME_WIDTH+30, 650);
        g.drawString("Total Time: " + df.format((velocities.size()-1) * 0.02) + " s", FRAME_WIDTH+30, 680);

        g.drawString("Adjust Time", FRAME_WIDTH+30, 730);

    }

    public void runSimulation() {
        if(!simulating) {
            if(runSimButton.getText().equals("RUN SIM")) {cur_pos_index = 0;}
            simulating = true;
            runSimButton.setText("STOP");
        } else {
            simulating = false;
            runSimButton.setText("CONTINUE");
        }
    }

    public void setTime(int ind) {
        if(ind != cur_pos_index) {
            simulating = false;
            runSimButton.setText("CONTINUE");
            cur_pos_index = ind;
        }
    }

    public void updateAdjustFrame() {
        DecimalFormat df = new DecimalFormat();
        df.setMaximumFractionDigits(4);

        rightFields[0].setText(df.format(path.getMaxAcceleration() * Path.TO_INCHES));
        rightFields[1].setText(df.format(path.getMaxDeceleration() * Path.TO_INCHES));
        rightFields[2].setText(df.format(path.getMaxVelocity() * Path.TO_INCHES));
        rightFields[3].setText(df.format(path.getMaxAngularVelocity() * Path.TO_INCHES));
        rightFields[4].setText(df.format(path.getStartVelocity() * Path.TO_INCHES));
        rightFields[5].setText(df.format(path.getEndVelocity() * Path.TO_INCHES));
        rightFields[6].setText(df.format(startPosition.getPosition().x * Path.TO_INCHES));
        rightFields[7].setText(df.format(startPosition.getPosition().y * Path.TO_INCHES));
        rightFields[8].setText(df.format(startPosition.getAngleRadians() * 180 / Math.PI));
        rightFields[9].setText(df.format(b));
        rightFields[10].setText(df.format(Z));
        rightFields[11].setText(df.format(END_THRESHOLD * Path.TO_INCHES));
        rightFields[12].setText(df.format(ADJUST_THRESHOLD * Path.TO_INCHES));
        rightFields[13].setText(df.format(NEWTONS_STEPS));

        leftFields[0].setText(df.format(path.getParametric().getPoint(0).x * Path.TO_INCHES));
        leftFields[1].setText(df.format(path.getParametric().getPoint(0).y * Path.TO_INCHES));
        leftFields[2].setText(df.format(path.getParametric().getAngle(0).getAngle() * 180 / Math.PI));
        leftFields[3].setText(df.format(path.getParametric().getPoint(1).x * Path.TO_INCHES));
        leftFields[4].setText(df.format(path.getParametric().getPoint(1).y * Path.TO_INCHES));
        leftFields[5].setText(df.format(path.getParametric().getAngle(1).getAngle() * 180 / Math.PI));
        leftFields[6].setText(df.format(path.getParametric().getDerivative(0, 1).x * Path.TO_INCHES));
        leftFields[7].setText(df.format(path.getParametric().getDerivative(0, 1).y * Path.TO_INCHES));
        leftFields[8].setText(df.format(path.getParametric().getDerivative(1, 1).x * Path.TO_INCHES));
        leftFields[9].setText(df.format(path.getParametric().getDerivative(1, 1).y * Path.TO_INCHES));
        leftFields[10].setText(df.format(path.getParametric().getDerivative(0, 2).x * Path.TO_INCHES));
        leftFields[11].setText(df.format(path.getParametric().getDerivative(0, 2).y * Path.TO_INCHES));
        leftFields[12].setText(df.format(path.getParametric().getDerivative(1, 2).x * Path.TO_INCHES));
        leftFields[13].setText(df.format(path.getParametric().getDerivative(1, 2).y * Path.TO_INCHES));

    }

    public void submitAdjustFrame() {

        String pose0X = leftFields[0].getText();
        String pose0Y = leftFields[1].getText();
        String pose0Angle = leftFields[2].getText();
        String pose1X = leftFields[3].getText();
        String pose1Y = leftFields[4].getText();
        String pose1Angle = leftFields[5].getText();
        String vel0X = leftFields[6].getText();
        String vel0Y = leftFields[7].getText();
        String vel1X = leftFields[8].getText();
        String vel1Y = leftFields[9].getText();
        String acc0X = leftFields[10].getText();
        String acc0Y = leftFields[11].getText();
        String acc1X = leftFields[12].getText();
        String acc1Y = leftFields[13].getText();

        Parametric newParametric = new QuinticHermiteSpline(new Pose2D(returnNumber(pose0X) * Path.TO_METERS, returnNumber(pose0Y) * Path.TO_METERS, returnNumber(pose0Angle) * Math.PI / 180),
                new Pose2D(returnNumber(pose1X) * Path.TO_METERS, returnNumber(pose1Y) * Path.TO_METERS, returnNumber(pose1Angle) * Math.PI / 180),
                new Vector2D(returnNumber(vel0X) * Path.TO_METERS, returnNumber(vel0Y) * Path.TO_METERS), new Vector2D(returnNumber(vel1X) * Path.TO_METERS, returnNumber(vel1Y) * Path.TO_METERS),
                new Vector2D(returnNumber(acc0X) * Path.TO_METERS, returnNumber(acc0Y) * Path.TO_METERS), new Vector2D(returnNumber(acc1X) * Path.TO_METERS, returnNumber(acc1Y) * Path.TO_METERS));

        String maxAcc = rightFields[0].getText();
        String maxDec = rightFields[1].getText();
        String maxVel = rightFields[2].getText();
        String maxAngVel = rightFields[3].getText();
        String stVel = rightFields[4].getText();
        String endVel = rightFields[5].getText();
        String startX = rightFields[6].getText();
        String startY = rightFields[7].getText();
        String startAng = rightFields[8].getText();

        this.startPosition = new Pose2D(returnNumber(startX) * Path.TO_METERS, returnNumber(startY) * Path.TO_METERS, returnNumber(startAng) * Math.PI / 180);

        path = new RamsetePath(newParametric, returnNumber(maxAcc) * Path.TO_METERS, returnNumber(maxDec) * Path.TO_METERS, returnNumber(maxVel) * Path.TO_METERS,
                returnNumber(maxAngVel) * Path.TO_METERS, returnNumber(stVel) * Path.TO_METERS, returnNumber(endVel) * Path.TO_METERS);

        b = returnNumber(rightFields[9].getText());
        Z = returnNumber(rightFields[10].getText());
        END_THRESHOLD = returnNumber(rightFields[11].getText()) * Path.TO_METERS;
        ADJUST_THRESHOLD = returnNumber(rightFields[12].getText()) * Path.TO_METERS;
        NEWTONS_STEPS = Integer.parseInt(rightFields[13].getText());
    }

    public int checkAll() {
        for(int i = 0; i < leftFields.length; i++) {
            if(checkAdjust(i, leftFields[i].getText(), false) == 1) {
                JOptionPane.showMessageDialog(adjustComponent, leftLabels[i] + " must be a number");
                return -1;
            }
        }

        for(int i = 0; i < rightFields.length; i++) {
            if(i == 13) {
                try {
                    Integer.parseInt(rightFields[i].getText());
                } catch (Exception e) {
                    JOptionPane.showMessageDialog(adjustComponent, rightLabels[i] + " must be an integer");
                    return -1;
                }
            }
            if(checkAdjust(i, rightFields[i].getText(), true) == 1) {
                JOptionPane.showMessageDialog(adjustComponent, rightLabels[i] + " must be a number");
                return -1;
            } else if (checkAdjust(i, rightFields[i].getText(), true) == 2 && i != 6 && i != 7) {
                JOptionPane.showMessageDialog(adjustComponent, rightLabels[i] + " must be positive");
                return -1;
            }
            if(i == 10) {
                if(returnNumber(rightFields[i].getText()) >= 1 || returnNumber(rightFields[i].getText()) <= 0) {
                    JOptionPane.showMessageDialog(adjustComponent, rightLabels[i] + " must be on the interval (0, 1)");
                    return -1;
                }
            }
        }
        return 0;
    }

    public int checkAdjust(int i, String s, boolean pos) {
        try {
            double d = Double.parseDouble(s);
            if(pos) {
                if(d < 0) {
                    return 2;
                }
            }
            return 0;
        } catch (Exception e) {
            return 1;
        }
    }

    public int exportPath() {

        if(checkAll() != 0) {
            return -1;
        }

        String pose0X = leftFields[0].getText();
        String pose0Y = leftFields[1].getText();
        String pose0Angle = leftFields[2].getText();
        String pose1X = leftFields[3].getText();
        String pose1Y = leftFields[4].getText();
        String pose1Angle = leftFields[5].getText();
        String vel0X = leftFields[6].getText();
        String vel0Y = leftFields[7].getText();
        String vel1X = leftFields[8].getText();
        String vel1Y = leftFields[9].getText();
        String acc0X = leftFields[10].getText();
        String acc0Y = leftFields[11].getText();
        String acc1X = leftFields[12].getText();
        String acc1Y = leftFields[13].getText();

        Parametric newParametric = new QuinticHermiteSpline(new Pose2D(returnNumber(pose0X) * Path.TO_METERS, returnNumber(pose0Y) * Path.TO_METERS, returnNumber(pose0Angle) * Math.PI / 180),
                new Pose2D(returnNumber(pose1X) * Path.TO_METERS, returnNumber(pose1Y) * Path.TO_METERS, returnNumber(pose1Angle) * Math.PI / 180),
                new Vector2D(returnNumber(vel0X) * Path.TO_METERS, returnNumber(vel0Y) * Path.TO_METERS), new Vector2D(returnNumber(vel1X) * Path.TO_METERS, returnNumber(vel1Y) * Path.TO_METERS),
                new Vector2D(returnNumber(acc0X) * Path.TO_METERS, returnNumber(acc0Y) * Path.TO_METERS), new Vector2D(returnNumber(acc1X) * Path.TO_METERS, returnNumber(acc1Y) * Path.TO_METERS));

        String maxAcc = rightFields[0].getText();
        String maxDec = rightFields[1].getText();
        String maxVel = rightFields[2].getText();
        String maxAngVel = rightFields[3].getText();
        String stVel = rightFields[4].getText();
        String endVel = rightFields[5].getText();
        String startX = rightFields[6].getText();
        String startY = rightFields[7].getText();
        String startAng = rightFields[8].getText();

        this.startPosition = new Pose2D(returnNumber(startX) * Path.TO_METERS, returnNumber(startY) * Path.TO_METERS, returnNumber(startAng) * Math.PI / 180);

        path = new RamsetePath(newParametric, returnNumber(maxAcc) * Path.TO_METERS, returnNumber(maxDec) * Path.TO_METERS, returnNumber(maxVel) * Path.TO_METERS,
                returnNumber(maxAngVel) * Path.TO_METERS, returnNumber(stVel) * Path.TO_METERS, returnNumber(endVel) * Path.TO_METERS);

        b = returnNumber(rightFields[9].getText());
        Z = returnNumber(rightFields[10].getText());
        END_THRESHOLD = returnNumber(rightFields[11].getText()) * Path.TO_METERS;
        ADJUST_THRESHOLD = returnNumber(rightFields[12].getText()) * Path.TO_METERS;
        NEWTONS_STEPS = Integer.parseInt(rightFields[13].getText());



        JFrame exportFrame = new JFrame();
        exportFrame.setSize(1000, 500 + TITLE_HEIGHT);
        exportFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        exportFrame.setResizable(false);

        JPanel panel = new JPanel();
        panel.setLayout(null);

        JTextArea f = new JTextArea();
        f.setEditable(false);
        f.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));
        f.setBounds(20, 20, 960, 460);
        f.setBackground(null);
        f.setBorder(null);
        f.setLineWrap(true);
        f.setWrapStyleWord(true);
        f.setText(
                "private double b = " + rightFields[9].getText() + ";\n" + "private double Z = " + rightFields[10].getText() + ";\nprivate double END_THRESHOLD = " + rightFields[11].getText() + " * Path.TO_METERS;\n" +
                "private double ADJUST_THRESHOLD = " + rightFields[12].getText() + " * Path.TO_METERS;\nprivate int NEWTON_STEPS = " + NEWTONS_STEPS + ";\n\n" +
                "QuinticHermiteSpline parametric = new QuinticHermiteSpline(\n    new Pose2D(" + pose0X + " * Path.TO_METERS, " +
                pose0Y + " * Path.TO_METERS, " + pose0Angle + " * Math.PI/180), \n    new Pose2D(" + pose1X + " * Path.TO_METERS, " +
                pose1Y + " * Path.TO_METERS, " + pose1Angle + " * Math.PI/180), \n    new Vector2D(" + vel0X + " * Path.TO_METERS, " +
                vel0Y + " * Path.TO_METERS), \n    new Vector2D(" + vel1X + " * Path.TO_METERS, " + vel1Y + " * Path.TO_METERS), \n    new Vector2D(" +
                acc0X + " * Path.TO_METERS, " + acc0Y + " * Path.TO_METERS), \n    new Vector2D(" + acc1X + " * Path.TO_METERS, " +
                acc1Y + " * Path.TO_METERS)\n);\n\n" +
                "RamsetePath path = new RamsetePath(parametric, \n    " + maxAcc + " * Path.TO_METERS, " + maxDec + " * Path.TO_METERS, \n    "
                + maxVel + " * Path.TO_METERS, " + maxAngVel + " * Path.TO_METERS, \n    " + stVel + " * Path.TO_METERS, " + endVel + " * Path.TO_METERS\n);"
        );
        panel.add(f);

        exportFrame.add(panel);

        exportFrame.setVisible(true);

        System.out.println("private double b = " + rightFields[9].getText() + ";\n" + "private double Z = " + rightFields[10].getText() + ";\nprivate double END_THRESHOLD = " + rightFields[11].getText() + " * Path.TO_METERS;\n" +
                "private double ADJUST_THRESHOLD = " + rightFields[12].getText() + " * Path.TO_METERS;\nprivate int NEWTON_STEPS = " + NEWTONS_STEPS + ";\n\n" +
                "QuinticHermiteSpline parametric = new QuinticHermiteSpline(\n    new Pose2D(" + pose0X + " * Path.TO_METERS, " +
                pose0Y + " * Path.TO_METERS, " + pose0Angle + " * Math.PI/180), \n    new Pose2D(" + pose1X + " * Path.TO_METERS, " +
                pose1Y + " * Path.TO_METERS, " + pose1Angle + " * Math.PI/180), \n    new Vector2D(" + vel0X + " * Path.TO_METERS, " +
                vel0Y + " * Path.TO_METERS), \n    new Vector2D(" + vel1X + " * Path.TO_METERS, " + vel1Y + " * Path.TO_METERS), \n    new Vector2D(" +
                acc0X + " * Path.TO_METERS, " + acc0Y + " * Path.TO_METERS), \n    new Vector2D(" + acc1X + " * Path.TO_METERS, " +
                acc1Y + " * Path.TO_METERS)\n);\n\n" +
                "RamsetePath path = new RamsetePath(parametric, \n    " + maxAcc + " * Path.TO_METERS, " + maxDec + " * Path.TO_METERS, \n    "
                + maxVel + " * Path.TO_METERS, " + maxAngVel + " * Path.TO_METERS, \n    " + stVel + " * Path.TO_METERS, " + endVel + " * Path.TO_METERS\n);");

        return 0;
    }

    public double returnNumber(String s) {
        return Double.parseDouble(s);
    }

    public void toggleAdjustFrame() {
        adjustFrame.setVisible(!adjustFrame.isVisible());
    }

    public int update() {

        if(checkAll() != 0) {
            return -1;
        }

        submitAdjustFrame();

        simulate(startPosition, END_TIME);

        toggleAdjustFrame();

        component.remove(timeSlider);
        timeSlider.setValue(0);

        updateTimeSlider();

        cur_pos_index = 0;
        simulating = false;
        runSimButton.setText("RUN SIM");

        component.updateUI();

        return 0;
    }

    private void updateTimeSlider() {
        timeSlider = new JSlider(JSlider.HORIZONTAL, 0, velocities.size()-1, 0);
        timeSlider.setPaintTicks(false);
        timeSlider.setPaintLabels(false);

        timeSlider.setBounds(FRAME_WIDTH+30, 750, 340, 20);

        timeSlider.addChangeListener(e -> setTime(timeSlider.getValue()));

        component.add(timeSlider);
    }

    public void run(Pose2D startPosition, double END_TIME) {

        this.startPosition = startPosition;
        this.END_TIME = END_TIME;

        simulate(startPosition, END_TIME);


        frame = new JFrame();

        TITLE_HEIGHT = 28;
        FRAME_WIDTH = 800;
        FRAME_HEIGHT = 800;
        ADJUST_FRAME_HEIGHT = 730;
        ADJUST_FRAME_WIDTH = 800;

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

        component.setLayout(null);
        frame.add(component);

        frame.setVisible(true);

        adjustFrame = new JFrame();
        adjustFrame.setSize(ADJUST_FRAME_WIDTH, ADJUST_FRAME_HEIGHT+TITLE_HEIGHT);
        adjustFrame.setResizable(false);
        adjustFrame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);

        adjustComponent = new JPanel();

        adjustComponent.setLayout(null);
        adjustFrame.add(adjustComponent);

        rightFields = new JTextField[rightLabels.length];

        for(int i = 0; i < rightLabels.length; i++) {
            rightFields[i] = new JTextField();
            rightFields[i].setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));

            rightFields[i].setBounds(ADJUST_FRAME_WIDTH * 3 / 4 + 10, 80 + 40 * (i), ADJUST_FRAME_WIDTH/4 - 80, 35);

            JLabel label = new JLabel(rightLabels[i], JLabel.RIGHT);
            label.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));
            label.setBounds(ADJUST_FRAME_WIDTH * 1 / 2 + 10, 80 + 40 * (i), ADJUST_FRAME_WIDTH / 4 - 20, 35);

            adjustComponent.add(label);
            adjustComponent.add(rightFields[i]);
        }

        leftFields = new JTextField[leftLabels.length];

        for(int i = 0; i < leftFields.length; i++) {
            leftFields[i] = new JTextField();
            leftFields[i].setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));

            leftFields[i].setBounds(ADJUST_FRAME_WIDTH * 1 / 4 + 50, 80 + 40 * (i), ADJUST_FRAME_WIDTH/4 - 80, 35);


            JLabel label = new JLabel(leftLabels[i], JLabel.RIGHT);
            label.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 16));
            label.setBounds(50, 80 + 40 * (i), ADJUST_FRAME_WIDTH / 4 - 20, 35);

            adjustComponent.add(label);
            adjustComponent.add(leftFields[i]);
        }

        JLabel l1 = new JLabel("Adjust Spline", JLabel.LEFT);
        l1.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 24));
        l1.setBounds(50, 30, ADJUST_FRAME_WIDTH/2-50, 30);

        adjustComponent.add(l1);

        JLabel l2 = new JLabel("Adjust Path", JLabel.LEFT);
        l2.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 24));
        l2.setBounds(ADJUST_FRAME_WIDTH/2 + 50, 30, ADJUST_FRAME_WIDTH/2-50, 30);

        adjustComponent.add(l2);

        updateButton = new JButton("Update");
        updateButton.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 24));
        updateButton.setBounds(ADJUST_FRAME_WIDTH/2-210, ADJUST_FRAME_HEIGHT-70, 200, 50);
        updateButton.addActionListener(e -> update());

        adjustComponent.add(updateButton);

        updateAdjustFrame();

        adjustFrame.setVisible(false);


        runSimButton = new JButton("RUN SIM");
        runSimButton.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 18));
        runSimButton.setBounds(FRAME_WIDTH+125, 100, 150, 50);
        runSimButton.addActionListener(e -> runSimulation());

        component.add(runSimButton);

        updateTimeSlider();

        adjustButton = new JButton("Adjust Path");
        adjustButton.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 18));
        adjustButton.setBounds(FRAME_WIDTH-175, 5, 150, 40);
        adjustButton.addActionListener(e -> toggleAdjustFrame());

        component.add(adjustButton);

        exportButton = new JButton("Export");
        exportButton.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 24));
        exportButton.setBounds(ADJUST_FRAME_WIDTH/2 + 10, ADJUST_FRAME_HEIGHT-70, 200, 50);
        exportButton.addActionListener(e -> exportPath());

        adjustComponent.add(exportButton);


        Runnable simRunnable = () -> {
            component.updateUI();

            if(simulating) {
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
                    runSimButton.setText("RUN SIM");
                    simulating = false;
                }
            }
        };

        executorService = Executors.newScheduledThreadPool(1);
        executorService.scheduleAtFixedRate(simRunnable, 0, 10, TimeUnit.MILLISECONDS);

    }

    public void simulate(Pose2D startPosition, double END_TIME) {

        cur_pos_index = 0;

        Vector2D maxPoint = path.getParametric().getAbsoluteMaxCoordinates(1000);
        double maxMeters = Math.max(maxPoint.getX(), maxPoint.getY()) + Math.max(TRACKLENGTH, TRACKWIDTH);

        METERS_TO_PIXELS = 400 / maxMeters;


        robotPosition = startPosition;
        cur_time = 0;

        curvatures.clear();
        robotPoses.clear();
        velocities.clear();
        angularVelocities.clear();
        linearVelocities.clear();
        parametrics.clear();

        vel.clear();
        angVel.clear();
        rvel.clear();
        rangVel.clear();
        ex.clear();
        ey.clear();
        et.clear();
        k.clear();
        t.clear();
        td.clear();



        curvatures.add(path.getCurvature(0));
        robotPoses.add(robotPosition);
        velocities.add(new Vector2D(path.getStartVelocity(), path.getStartVelocity()));
        angularVelocities.add(path.getAngularVelocityAtPoint(0, path.getStartVelocity()));
        linearVelocities.add(path.getStartVelocity());
        parametrics.add(path.getParametric());


        vel.add(0.);
        angVel.add(0.);
        rvel.add(0.);
        rangVel.add(0.);
        et.add(0.);
        ey.add(0.);
        ex.add(0.);
        k.add(0.);
        t.add(0.);
        td.add(0.);

        //http://rossum.sourceforge.net/papers/DiffSteer/

        while(cur_time < END_TIME) {

            DifferentialDriveState dds = path.update(robotPosition, dt, END_THRESHOLD, ADJUST_THRESHOLD, NEWTONS_STEPS, b, Z, TRACKWIDTH);

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

            curvatures.add(path.getCurvature());

            robotPosition = new Pose2D(new_x, new_y, newAngle);

            robotPoses.add(robotPosition);
            velocities.add(new Vector2D(dds.getLeftVelocity(), dds.getRightVelocity()));
            angularVelocities.add(dds.getAngularVelocity());
            linearVelocities.add(dds.getLinearVelocity());
            parametrics.add(path.getParametric());


            rvel.add(RamseteController.rvel);
            rangVel.add(RamseteController.angVel);
            vel.add(RamseteController.vel);
            angVel.add(RamseteController.angVel);
            ex.add(RamseteController.ex);
            ey.add(RamseteController.ey);
            et.add(RamseteController.et);
            k.add(RamseteController.k);
            t.add(RamseteController.t.getAngle());
            td.add(RamseteController.td.getAngle());



            if(path.isFinished(robotPosition, END_THRESHOLD)) {
                break;
            }

        }
    }

    public void run() {
        this.run(path.getParametric().getPose(0), 180);
    }

    public int convertXToPixels(double x) {
        return FRAME_WIDTH/2 + (int) (x * METERS_TO_PIXELS);
    }

    public int convertYToPixels(double y) {
        return FRAME_HEIGHT/2 - (int) (y * METERS_TO_PIXELS);
    }

}
