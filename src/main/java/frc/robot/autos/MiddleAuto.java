package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootCommand.SpeedLevel;
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
        Rotation2d robotHeading = Rotation2d.fromDegrees(-90);
        Rotation2d travelDirection = Rotation2d.fromDegrees(180);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(2.0, 4.0, travelDirection),
            new Pose2d(1.0, 4.0, travelDirection)
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720)),
            null,
            new GoalEndState(0.0, robotHeading)
        );
        path.preventFlipping = true;

        Pose2d startPose = new Pose2d(2.0, 4.0, robotHeading);

        addCommands(
            Commands.runOnce(() -> drivetrain.resetPose(startPose)),
            AutoBuilder.followPath(path),
            Commands.runOnce(() -> intake.setGoal(Intake.Goal.DEPLOY), intake),
            Commands.runOnce(() -> index.setIndexState(IndexState.ACTIVE), index),
            Commands.runOnce(() -> {
                turret.setGoal(Turret.Goal.HUB);
                turret.setAngle(new Rotation2d(Math.PI / 2));
            }, turret),
            new ShootCommand(shooter, hood, SpeedLevel.LOW).withTimeout(15.0),
            new WaitCommand(2),
            Commands.runOnce(() -> index.setIndexState(IndexState.STOP), index)
        );
    }
}
