package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCommand.SpeedLevel;
import frc.robot.commands.TransitCommand;
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
        try{
            Pose2d startPose = new Pose2d(3.6, 4.0, Rotation2d.fromDegrees(180));

            PathPlannerPath path = PathPlannerPath.fromPathFile("hubToShoot");
            PathPlannerPath path1 = PathPlannerPath.fromPathFile("toDepot");
            PathPlannerPath path2 = PathPlannerPath.fromPathFile("depotIntake");
            PathPlannerPath path3 = PathPlannerPath.fromPathFile("toShoot");
            PathPlannerPath path4 = PathPlannerPath.fromPathFile("toOutside");

            path.preventFlipping = true;
            path1.preventFlipping = true;
            path2.preventFlipping = true;
            path3.preventFlipping = true;
            path4.preventFlipping = true;

            addCommands(
                Commands.parallel(
                    new ShootCommand(shooter, hood, ShootCommand.SpeedLevel.LOW).withTimeout(19.0), 
                    Commands.sequence(
                        Commands.runOnce(() -> drivetrain.resetPose(startPose)),
                        Commands.runOnce(() -> {
                            turret.setGoal(Turret.Goal.HUB);
                            turret.setAngle(new Rotation2d(-Math.PI));
                        }, turret), 
                        AutoBuilder.followPath(path),
                        Commands.runOnce(() -> intake.setGoal(Intake.Goal.DEPLOY), intake),
                        new TransitCommand(intake, index).withTimeout(4.0),
                        Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE), intake),
                        AutoBuilder.followPath(path1),
                        AutoBuilder.followPath(path2),
                        AutoBuilder.followPath(path3),
                        new TransitCommand(intake, index).withTimeout(4.5),
                        AutoBuilder.followPath(path4)
                    )
                )
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Left auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
