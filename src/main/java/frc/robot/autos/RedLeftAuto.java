package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RedLeftAuto extends SequentialCommandGroup {
    private static final String[] PATHS = {
        "RL_StartToIntake1",
        "RL_IntakeToShoot1",
        "RL_ShootToIntake2",
        "RL_IntakeToShoot2"
    };

    public RedLeftAuto() {
        try {
            for (String name : PATHS) {
                addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile(name)));
            }
        } catch (Exception e) {
            DriverStation.reportError(
                "Failed to load RedLeft auto: " + e.getMessage(), e.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
