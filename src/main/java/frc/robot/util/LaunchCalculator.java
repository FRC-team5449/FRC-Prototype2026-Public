package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LaunchCalculator {

    /**
     * Turret offset from chassis center to turret rotation center (meters).
     * Positive means turret is in front of chassis center. Set to 0 to ignore.
     */
    private static final double TURRET_OFFSET = 0.0;

    /**
     * Calculate the turret angle in robot-relative coordinates.
     * Ported from FTC Util.goalInTurretSys():
     * 1. Compute actual turret position (accounting for chassis-to-turret offset)
     * 2. Compute target vector (dx, dy) relative to turret in field coordinates
     * 3. Rotate vector from field frame to robot frame
     * 4. Use atan2 to get the angle the turret should point at
     */
    public static Rotation2d calculateTurretAngle(Pose2d robotPose, Translation2d target) {
        double heading = robotPose.getRotation().getRadians();

        double turretX = robotPose.getX() + Math.cos(heading) * TURRET_OFFSET;
        double turretY = robotPose.getY() + Math.sin(heading) * TURRET_OFFSET;

        double dx = target.getX() - turretX;
        double dy = target.getY() - turretY;

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double localX = dx * cosH + dy * sinH;
        double localY = -dx * sinH + dy * cosH;

        return new Rotation2d(localX, localY);
    }

    /**
     * Calculate shooter speed (rps) from distance to target.
     * Currently a linear approximation; calibrate with real data later.
     */
    public static double calculateRPM(double distance) {
        return -8 + distance * -15.5;
    }

    /**
     * Calculate distance from robot to target (meters).
     */
    public static double calculateDistance(Pose2d robotPose, Translation2d target) {
        return robotPose.getTranslation().getDistance(target);
    }
}
