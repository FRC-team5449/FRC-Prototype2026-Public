package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TransitCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class LeftAuto extends SequentialCommandGroup {

    public LeftAuto(Shooter shooter, Turret turret, Index index, Hood hood,
                    CommandSwerveDrivetrain drivetrain, Intake intake) {
        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("BL_StartToIntake1");
            PathPlannerPath path2 = PathPlannerPath.fromPathFile("BL_IntakeToShoot1");
            PathPlannerPath path3 = PathPlannerPath.fromPathFile("BL_ShootToIntake2");
            PathPlannerPath path4 = PathPlannerPath.fromPathFile("BL_IntakeToShoot2");

            path1.preventFlipping = true;
            path2.preventFlipping = true;
            path3.preventFlipping = true;
            path4.preventFlipping = true;

            addCommands(
                Commands.runOnce(() -> {
                    Pose2d startPose = new Pose2d(3.5, 7.4, new Rotation2d(Math.toRadians(0)));
                    drivetrain.resetPose(startPose);
                }),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.DEPLOY), intake),
                new WaitCommand(1.0),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE), intake),
                AutoBuilder.followPath(path1),
                Commands.runOnce(() -> {
                    turret.setGoal(Turret.Goal.HUB);
                    turret.setAngle(new Rotation2d(Math.toRadians(-71.5)));
                }, turret),
                AutoBuilder.followPath(path2),
                new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.MEDIUM).withTimeout(5.5)
                .alongWith(new WaitCommand(1.0).andThen(
                new TransitCommand(intake, index).withTimeout(4.5))),
                Commands.runOnce(() -> index.setIndexState(IndexState.STOP), index),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE), intake),
                AutoBuilder.followPath(path3),
                AutoBuilder.followPath(path4),
                new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.MEDIUM).withTimeout(5.5)
                .alongWith(new WaitCommand(1.0).andThen(
                new TransitCommand(intake, index).withTimeout(4.5)))
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Left auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
