package frc.robot.util;

import static edu.wpi.first.units.Units.derive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LaunchCalculator {
    public static Rotation2d calculateTurretAngle(Pose2d robotPose, Translation2d target) {
        Translation2d diff = target.minus(robotPose.getTranslation());
        return new Rotation2d(diff.getX(), diff.getY());
    }

    public static double calculateRPM(double distance, double velocityTowardTarget) {
        return 1 + distance * 1 - velocityTowardTarget * 1;

    }
}
