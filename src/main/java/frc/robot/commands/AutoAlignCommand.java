package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LaunchCalculator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Continuous turret auto-aim command. Reads chassis pose every cycle and
 * calculates the turret angle to keep the shooter facing the hub.
 * Ported from FTC TurretAlignCommandStatic.
 *
 * Only requires turret (not drivetrain), so the driver can still control
 * the chassis while this command runs. Designed to be used as turret's
 * default command for always-on tracking during teleop.
 */
public class AutoAlignCommand extends Command {
    private final Turret turret;
    private final CommandSwerveDrivetrain drivetrain;

    public AutoAlignCommand(Turret turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setGoal(Turret.Goal.HUB);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;

        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.Hub.blueTopCenterPoint
            : FieldConstants.Hub.redTopCenterPoint;

        Rotation2d turretAngle = LaunchCalculator.calculateTurretAngle(robotPose, target);
        double distance = LaunchCalculator.calculateDistance(robotPose, target);

        turret.setAngle(turretAngle);

        Logger.recordOutput("AutoAlign/TurretAngleDeg", turretAngle.getDegrees());
        Logger.recordOutput("AutoAlign/DistanceToTarget", distance);
    }

    @Override
    public void end(boolean interrupted) {
        turret.setGoal(Turret.Goal.STOP);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
