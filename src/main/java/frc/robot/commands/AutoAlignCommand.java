package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.shooter.ShooterCalculator;

import edu.wpi.first.math.geometry.*;

public class AutoAlignCommand extends Command {

    private final Shooter shooter;
    private final Turret turret;
    private final CommandSwerveDrivetrain drivetrain;

    public AutoAlignCommand(Shooter shooter, Turret turret, CommandSwerveDrivetrain drivetrain) {
        this.shooter = shooter;
        this.turret = turret;
        this.drivetrain = drivetrain;

        addRequirements(shooter, turret, drivetrain);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;

        Translation2d target = FieldConstants.Hub.topCenterPoint;

        double distance =
                robotPose.getTranslation().getDistance(target);

        Rotation2d turretAngle = ShooterCalculator.calculateTurretAngle(robotPose, target);

        double rpm = ShooterCalculator.calculateRPM(distance);

        turret.setAngle(turretAngle);
        shooter.setTarget(rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}