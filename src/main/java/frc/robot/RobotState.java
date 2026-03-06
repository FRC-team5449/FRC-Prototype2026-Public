package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.DriveConstants;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics swerveDriveKinematics;

    private RobotState() {
        swerveDriveKinematics = new SwerveDriveKinematics(DriveConstants.moduleTranslations);
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        swerveDriveKinematics,
                        new Rotation2d(),
                        new SwerveModulePosition[] {
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition()
                        },
                        new Pose2d());
    }

    /** update odometry */
    public void addOdometryObservation(
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions) {

        poseEstimator.update(
                gyroAngle,
                modulePositions);
    }

    /** add Limelight vision pose */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {

        poseEstimator.addVisionMeasurement(
                visionPose,
                timestamp);
    }

    /** get robot pose */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** reset pose */
    public void resetPose(Pose2d pose) {

        poseEstimator.resetPosition(
                pose.getRotation(),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition()},
                pose);
    }
}