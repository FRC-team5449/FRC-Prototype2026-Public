package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        addCommands(
            Commands.runOnce(() -> index.setIndexState(IndexState.ACTIVE), index),
            new ShootCommand(shooter, hood, SpeedLevel.LOW).withTimeout(20.0),
            Commands.runOnce(() -> index.setIndexState(IndexState.STOP), index)
        );
    }
}
