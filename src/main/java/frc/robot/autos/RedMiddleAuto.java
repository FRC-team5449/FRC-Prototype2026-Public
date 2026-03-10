package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedMiddleAuto extends SequentialCommandGroup {
    public RedMiddleAuto() {
        try {
            addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("RM_StartToFinish")));
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load RedMiddle auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
