package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueRightLoadingAuto extends SequentialCommandGroup {
    public BlueRightLoadingAuto() {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("BlueRightLoading");
            addCommands(AutoBuilder.followPath(path));
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load BlueRightLoading path: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
