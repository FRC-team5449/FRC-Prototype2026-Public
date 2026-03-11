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
    private final Turret turret;
    private final CommandSwerveDrivetrain drivetrain;

    public AutoAlignCommand(Turret turret, CommandSwerveDrivetrain drivetrain, Hood hood) {
        this.turret = turret;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;

        Translation2d target = DriverStation.getAlliance().get() == Alliance.Blue? 
            FieldConstants.Hub.blueTopCenterPoint: FieldConstants.Hub.redTopCenterPoint;

        // double distance = robotPose.getTranslation().getDistance(target);


        Rotation2d turretAngle = LaunchCalculator.calculateTurretAngle(robotPose, target);

        Translation2d toTarget = target.minus(robotPose.getTranslation());


        // double rpm = LaunchCalculator.calculateRPM(distance);

        turret.setAngle(turretAngle);
        // shooter.setTarget(rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}