package path;

import math.Angle;
import math.Pose2D;
import math.Vector2D;

public class RamseteController {

    public static double ex, ey, et, k, vel, angVel, rvel, rAngVel;
    public static Angle t, td;

    public static DifferentialDriveState ramsete(Pose2D curPose, Pose2D desiredPose, double desiredVelocity, double desiredAngularVelocity, double b, double Z, double trackwidth) {
        //https://file.tavsys.net/control/controls-engineering-in-frc.pdf

        //b > 0, 0 < Z < 1, larger b -> faster convergence, larger Z -> more dampening

        vel = desiredVelocity;
        angVel = desiredAngularVelocity;


        k = 2*Z*Math.sqrt(desiredAngularVelocity*desiredAngularVelocity + b * desiredVelocity * desiredVelocity);

        t = curPose.getAngle();
        td = desiredPose.getAngle();
        double x = curPose.getPosition().x;
        double xd = desiredPose.getPosition().x;
        double y = curPose.getPosition().y;
        double yd = desiredPose.getPosition().y;

        ex = t.cos() * (xd - x) + t.sin() * (yd - y);
        ey = t.sin() * (x - xd) + t.cos() * (yd - y);
        et = td.getAngleBetween(t);

//        System.out.println("ex: " + ex + " | ey: " + ey + " | et: " + et);

//        System.out.println("des velocity: " + desiredVelocity + " | des ang: " + desiredAngularVelocity);

        rvel = desiredVelocity * Math.cos(et) + k * ex;
        rAngVel = desiredAngularVelocity + k * et + b * desiredVelocity * sinc(et) * ey;


//        if(desiredAngularVelocity == 0) {
//            System.out.println("velocity: " + velocity + " | angularvelocity: " + angularVelocity);
//        }
        DifferentialDriveState dds = new DifferentialDriveState(trackwidth);

        dds.updateFromLinearAndAngularVelocity(rvel, rAngVel, trackwidth);
        return dds;
    }

    public static double sinc(double e) {
        if(e == 0) return 1;
        return Math.sin(e)/e;
    }
}
