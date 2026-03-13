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

public class RightAuto extends SequentialCommandGroup {

    public RightAuto(Shooter shooter, Turret turret, Index index, Hood hood,
                    CommandSwerveDrivetrain drivetrain, Intake intake) {
        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("BR/BR_StartToIntake1");

            path1.preventFlipping = true;

            addCommands(
                Commands.runOnce(() -> {
                    Pose2d startPose = path1.getStartingHolonomicPose().get();
                    drivetrain.resetPose(startPose);
                }),
                Commands.run(() -> intake.setGoal(Intake.Goal.DEPLOY), intake),
                new WaitCommand(1.5),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE), intake),
                AutoBuilder.followPath(path1),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR/BR_IntakeToShoot1")),
                Commands.runOnce(() -> {
                    turret.setGoal(Turret.Goal.HUB);
                    turret.setAngle(new Rotation2d(Math.toRadians(-71)));
                }, turret),
                new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.MEDIUM).withTimeout(7.0),
                new WaitCommand(2.0),
                new TransitCommand(intake, index).withTimeout(5.0),
                Commands.runOnce(() -> index.setIndexState(IndexState.STOP)),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR/BR_ShootToIntake2")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR/BR_IntakeToShoot2")),
                new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.MEDIUM).withTimeout(7.0),
                new WaitCommand(2.0), 
                new TransitCommand(intake, index).withTimeout(5.0)
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Left auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
