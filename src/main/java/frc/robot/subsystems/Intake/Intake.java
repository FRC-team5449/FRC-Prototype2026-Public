package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    public enum IntakeState {
        FORWARD(0.5),
        STOP(0.0);

        public double power;

        IntakeState(double power) {
            this.power = power;
        }
    }
    private IntakeState intakeState;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorCanId, IntakeConstants.intakeCanBus);
        intakeState = IntakeState.STOP;
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    @Override
    public void periodic() {
        intakeMotor.set(intakeState.power);
    }
}
