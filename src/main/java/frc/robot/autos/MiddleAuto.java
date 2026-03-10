package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.intake.Intake;

public class MiddleAuto extends SequentialCommandGroup {
    public MiddleAuto(/* Intake intake */) {
        try {
            addCommands(
                // Commands.runOnce(() -> intake.setGoal(Intake.Goal.MIDDLE)),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("BM_StartToFinish"))
            );
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load Middle auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
