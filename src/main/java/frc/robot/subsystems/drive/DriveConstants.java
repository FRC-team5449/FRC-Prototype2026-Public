package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotType;

public class DriveConstants {
    public static final double trackWidthX = Units.inchesToMeters(21.522);
    public static final double trackWidthY = Units.inchesToMeters(21.522);
    public static final Translation2d[] moduleTranslations = {
        new Translation2d(trackWidthX / 2, trackWidthY / 2),
        new Translation2d(trackWidthX / 2, -trackWidthY / 2),
        new Translation2d(-trackWidthX / 2, trackWidthY / 2),
        new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
    };
}
