package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable table;

    private Pose2d latestPose = new Pose2d();
    private double latestTimestamp = 0.0;
    private boolean hasNewResult = false;

    public VisionSubsystem(String limelightName) {
        table = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    @Override
    public void periodic() {
        hasNewResult = false;

        long tvRaw = table.getEntry("tv").getInteger(0);
        boolean hasTarget = tvRaw == 1;
        Logger.recordOutput("Vision/tv_raw", tvRaw);
        Logger.recordOutput("Vision/HasTarget", hasTarget);

        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);
        Logger.recordOutput("Vision/botpose_length", botpose.length);

        if (!hasTarget || botpose.length < 7) return;

        Logger.recordOutput("Vision/botpose_x", botpose[0]);
        Logger.recordOutput("Vision/botpose_y", botpose[1]);
        Logger.recordOutput("Vision/botpose_yaw", botpose[5]);
        Logger.recordOutput("Vision/botpose_latencyMs", botpose[6]);

        double latencySeconds = botpose[6] / 1000.0;
        latestTimestamp = Timer.getFPGATimestamp() - latencySeconds;

        latestPose = new Pose2d(
            botpose[0],
            botpose[1],
            Rotation2d.fromDegrees(botpose[5])
        );

        hasNewResult = true;

        Logger.recordOutput("Vision/Pose", latestPose);
    }

    public boolean hasNewResult() {
        return hasNewResult;
    }

    public Pose2d getLatestPose() {
        return latestPose;
    }

    public double getLatestTimestamp() {
        return latestTimestamp;
    }
}
