package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommand extends Command {
    private final Climber climber;

    public ClimberCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
