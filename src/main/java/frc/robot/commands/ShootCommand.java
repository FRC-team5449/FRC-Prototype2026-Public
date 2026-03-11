package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.Goal;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final boolean highSpeed;

    public ShootCommand(Shooter shooter, boolean highSpeed) {
        this.shooter = shooter;
        this.highSpeed = highSpeed;
    }

    @Override
    public void initialize() {
        shooter.setGoal(Goal.HUB);
    }

    @Override
    public void execute() {
        if (highSpeed) shooter.setTarget(-80);
        else shooter.setTarget(-60);
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.setTarget(0);
    }
}
