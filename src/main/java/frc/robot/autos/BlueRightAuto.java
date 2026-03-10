package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BlueRightAuto extends SequentialCommandGroup {
    private static final String[] PATHS = {
        "BR_StartToIntake1",
        "BR_IntakeToShoot1",
        "BR_ShootToIntake2",
        "BR_IntakeToShoot2"
    };

    public BlueRightAuto() {
        try {
            for (String name : PATHS) {
                addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(name)));
            }
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load BlueRight auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
