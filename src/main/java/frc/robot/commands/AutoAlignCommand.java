package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.Hood.Position;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Goal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.LaunchCalculator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignCommand extends Command {

    private final Shooter shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final Hood hood;

    public AutoAlignCommand(Shooter shooter, CommandSwerveDrivetrain drivetrain, Hood hood) {
        this.shooter = shooter;
        // this.turret = turret;
        this.drivetrain = drivetrain;
        this.hood = hood;

        addRequirements(drivetrain);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setGoal(Goal.HUB);
        hood.setPosition(Position.MIDDLE);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;

        Translation2d target = DriverStation.getAlliance().get() == Alliance.Blue? 
            FieldConstants.Hub.blueTopCenterPoint: FieldConstants.Hub.redTopCenterPoint;

        double distance = robotPose.getTranslation().getDistance(target);

        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        Translation2d lead = target.minus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).times(1));

        Rotation2d turretAngle = LaunchCalculator.calculateTurretAngle(robotPose, lead);

        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        Translation2d direction = toTarget.div(toTarget.getNorm());
        double velocityTowardTarget = robotVel.dot(direction);

        double rpm = LaunchCalculator.calculateRPM(distance, velocityTowardTarget);

        // turret.setAngle(turretAngle);
        // shooter.setTarget(rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}