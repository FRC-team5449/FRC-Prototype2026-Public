package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeMotorState;

public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.setIntakeMotorState(IntakeMotorState.FORWARD);
    }

    @Override
    public void end(boolean isInterrupted) {
        intake.setIntakeMotorState(IntakeMotorState.STOP);
    }
}
