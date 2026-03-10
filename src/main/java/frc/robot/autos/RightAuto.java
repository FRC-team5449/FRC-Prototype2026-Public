package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.intake.Intake;

public class RightAuto extends SequentialCommandGroup {
    public RightAuto(/* Intake intake */) {
        try {
            addCommands(
                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR_StartToIntake1")),

                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.MIDDLE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR_IntakeToShoot1")),

                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR_ShootToIntake2")),

                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.MIDDLE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BR_IntakeToShoot2"))
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Right auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
