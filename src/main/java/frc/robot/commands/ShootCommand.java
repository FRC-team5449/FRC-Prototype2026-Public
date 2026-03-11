package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Goal;

public class ShootCommand extends Command {
    public enum SpeedLevel {
        LOW(-40, Hood.Position.DOWN),
        MEDIUM(-60, Hood.Position.MIDDLE),
        HIGH(-80, Hood.Position.UP);

        public final double rpm;
        public final Hood.Position hoodPosition;

        SpeedLevel(double rpm, Hood.Position hoodPosition) {
            this.rpm = rpm;
            this.hoodPosition = hoodPosition;
        }
    }

    private final Shooter shooter;
    private final Hood hood;
    private final SpeedLevel speedLevel;

    public ShootCommand(Shooter shooter, Hood hood, SpeedLevel speedLevel) {
        this.shooter = shooter;
        this.hood = hood;
        this.speedLevel = speedLevel;
        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
        shooter.setGoal(Goal.HUB);
        hood.setPosition(speedLevel.hoodPosition);
    }

    @Override
    public void execute() {
        shooter.setTarget(speedLevel.rpm);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTarget(0);
    }
}
