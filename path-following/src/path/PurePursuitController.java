package path;

import math.Circle;
import math.Point2D;
import math.Pose2D;

public class PurePursuitController {
    public static DifferentialDriveState purePursuit(Pose2D robotPose, Point2D lookaheadPoint, double linearVelocity, double trackwidth) {
        DifferentialDriveState dds = new DifferentialDriveState(trackwidth);
        Circle tangentCircle = new Circle();
        tangentCircle.fromTangentAndPoint(robotPose, lookaheadPoint);

        dds.fromLinearAndRadius(linearVelocity, tangentCircle.getRadius(), trackwidth);

        return dds;
    }
}
