// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.LeftAuto;
import frc.robot.autos.MiddleAuto;
import frc.robot.autos.RightAuto;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.TransitCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hood.Hood;

public class RobotContainer {
    private final Intake intake;
    private final Climber climber;
    private final Index index;
    private final Shooter shooter;
    private final Turret turret;
    private final Hood hood;

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // 在 RobotContainer 构造函数中
    private final VisionSubsystem vision;

    public RobotContainer() {
        intake = new Intake();
        climber = new Climber();
        index = new Index();
        shooter = new Shooter();
        turret = new Turret();
        hood = new Hood();
        vision = new VisionSubsystem("limelight");

        drivetrain.setVisionSubsystem(vision);

        configureBindings();
        configureAutos();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.circle().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.circle().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding create/options and square/triangle.
        // Note that each routine should be run exactly once in a single log.
        // joystick.create().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.create().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on options press.
        joystick.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.L2().whileTrue(
            Commands.run(() -> intake.setGoal(Intake.Goal.MIDDLE), intake)
                .finallyDo(() -> intake.setGoal(Intake.Goal.DEPLOY))
        );
        joystick.R2().onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.RETRACT), intake));

        joystick.L1().whileTrue(new TransitCommand(intake, index));

        joystick.R1().onTrue(Commands.run(() -> {
            shooter.setTarget(-60.0);
            shooter.setGoal(Shooter.Goal.HUB);
        }, shooter));

        joystick.R1().onFalse(Commands.run(() -> {
            shooter.setTarget(0);
            shooter.setGoal(Shooter.Goal.STOP);
        }, shooter));
        
        // joystick.triangle().onTrue(climber.toggleCommand());

        joystick.square().onTrue(Commands.runOnce(() -> {
            double current = hood.getPosition();
            if (current < 3.0) {
                hood.setPosition(Hood.Position.MIDDLE);
            } else if (current < 10.0) {
                hood.setPosition(Hood.Position.UP);
            } else {
                hood.setPosition(Hood.Position.DOWN);
            }
        }, hood));

        joystick.cross().onTrue(
            Commands.run(() -> new AutoAlignCommand(shooter, turret, drivetrain))
        );
    }

    private void configureAutos() {
        autoChooser.setDefaultOption("Left", new LeftAuto(/* intake */));
        autoChooser.addOption("Right", new RightAuto(/* intake */));
        autoChooser.addOption("Middle", new MiddleAuto(/* intake */));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
