package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Goal;

public class TransitCommand extends Command {
    private final Intake intake;
    private final Index index;

    private boolean onMiddle;
    private double lastTime;

    public TransitCommand(Intake intake, Index index) {
        this.intake = intake;
        this.index = index;

        onMiddle = false;
        lastTime = Timer.getFPGATimestamp();

        addRequirements(intake);
        addRequirements(index);
    }

    @Override
    public void initialize() {
        intake.setGoal(Goal.DEPLOY);
        index.setIndexState(IndexState.ACTIVE);
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        if (now - lastTime >= 1.0) {
            if (onMiddle) {
                intake.setGoal(Goal.DEPLOY);
                onMiddle = false;
            }
            else {
                intake.setGoal(Goal.MIDDLE);
                onMiddle = true;
            }

            lastTime = now;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setGoal(Goal.DEPLOY);
        index.setIndexState(IndexState.STOP);
    }
}
