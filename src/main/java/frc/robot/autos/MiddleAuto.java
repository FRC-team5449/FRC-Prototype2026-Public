package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class MiddleAuto extends SequentialCommandGroup {
    public MiddleAuto(Shooter shooter, Turret turret, Index index, Hood hood,
                      CommandSwerveDrivetrain drivetrain, Intake intake) {
        try {
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("BM_StartToFinish");

            addCommands(
                Commands.runOnce(() -> {
                    Pose2d startPose = path1.getStartingHolonomicPose().orElse(new Pose2d());
                    drivetrain.resetPose(startPose);
                }),

                // --- Start: turret lock → shoot → feed (preloaded) ---
                // new AutoAlignCommand(turret, drivetrain).withTimeout(1.5),
                // new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.HIGH).withTimeout(1.0),
                // Commands.runOnce(() -> index.setState(IndexState.ACTIVE)),
                new WaitCommand(0.5),
                // Commands.runOnce(() -> index.setState(IndexState.STOP)),

                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                AutoBuilder.followPath(path1),

                new WaitCommand(2.0),
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.DEPLOY))

                // --- End: turret lock → shoot → feed ---
                // new AutoAlignCommand(turret, drivetrain).withTimeout(1.5),
                // new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.HIGH).withTimeout(1.0),
                // Commands.runOnce(() -> index.setState(IndexState.ACTIVE)),
                // new WaitCommand(0.5),
                // Commands.runOnce(() -> index.setState(IndexState.STOP))
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Middle auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
