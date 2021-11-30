package path;

import math.Circle;
import math.Point2D;
import math.Pose2D;

public class PurePursuitController {
    public static DifferentialDriveState purePursuit(Pose2D robotPose, Point2D lookaheadPoint, double linearVelocity, double trackwidth) {
        DifferentialDriveState dds = new DifferentialDriveState(trackwidth);
        Circle tangentCircle = new Circle();
        tangentCircle.updateFromPoseAndPoint(robotPose, lookaheadPoint);

        dds.updateFromLinearVelocityAndRadius(linearVelocity, tangentCircle.getRadius(), trackwidth);

        return dds;
    }

    public static DifferentialDriveState purePursuit(double tangentRadius, double linearVelocity, double trackwidth) {
        DifferentialDriveState dds = new DifferentialDriveState(trackwidth);
        dds.updateFromLinearVelocityAndRadius(linearVelocity, tangentRadius, trackwidth);
        return dds;
    }
}
