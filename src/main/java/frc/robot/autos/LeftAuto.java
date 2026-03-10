package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.intake.Intake;

public class LeftAuto extends SequentialCommandGroup {
    public LeftAuto(/* Intake intake */) {
        try {
            addCommands(
                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BL_StartToIntake1")),

                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.MIDDLE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BL_IntakeToShoot1")),

                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BL_ShootToIntake2")),

                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.MIDDLE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BL_IntakeToShoot2"))
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Left auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
