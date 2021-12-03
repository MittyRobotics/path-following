package path;

import math.Circle;
import math.Point2D;
import math.Pose2D;

public class PurePursuitController {
    public static DifferentialDriveState purePursuit(double tangentRadius, double linearVelocity, boolean turnRight, double trackwidth) {
        DifferentialDriveState dds = new DifferentialDriveState(trackwidth);
        dds.updateFromLinearVelocityAndRadius(linearVelocity, tangentRadius, turnRight, trackwidth);
        return dds;
    }
}
