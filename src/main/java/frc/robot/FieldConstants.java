package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    public static final double fieldWidth = field.getFieldWidth();

    public class Hub {
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(72);
        public static final Translation2d topCenterPoint = new Translation2d(
            field.getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0);
        
    }
}
