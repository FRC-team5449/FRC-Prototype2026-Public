package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterCalculator {
    public static Rotation2d calculateTurretAngle(Pose2d robotPose, Translation2d target) {
        Translation2d diff = target.minus(robotPose.getTranslation());
        return new Rotation2d(diff.getX(), diff.getY());
    }

    public static double calculateRPM(double distance) {
        return 3000 + distance * 500;

    }
}
