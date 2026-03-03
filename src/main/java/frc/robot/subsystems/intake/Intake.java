package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor, intakeArm;
    private final MotionMagicVoltage motionMagicVoltage;

    public enum IntakeMotorState {
        FORWARD(0.8),
        STOP(0.0);

        public double power;

        IntakeMotorState(double power) {
            this.power = power;
        }
    }

    private IntakeMotorState intakeMotorState;

    public enum IntakeArmState {
        FORWARD(0.8),
        BACK(0.0);

        public double pos;

        IntakeArmState(double pos) {
            this.pos = pos;
        }
    }

    private IntakeArmState intakeArmState;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorCanId, IntakeConstants.intakeCanBus);
        intakeArm = new TalonFX(IntakeConstants.intakeArmCanId, IntakeConstants.intakeCanBus);
        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
        intakeArm.getConfigurator().apply(IntakeConstants.getConfigs());

        intakeMotorState = IntakeMotorState.STOP;
        intakeArmState = IntakeArmState.BACK;
    }

    // public void setIntakeMotorState(IntakeMotorState intakeMotorState) {
    //     this.intakeMotorState = intakeMotorState;
    // }

    // public IntakeMotorState getIntakeMotorState() {
    //     return intakeMotorState;
    // }

    public Command setIntakeMotorState(IntakeMotorState intakeMotorState) {
        return startEnd(() -> this.intakeMotorState = intakeMotorState, () -> {});
    }

    public Command setIntakeArmState(IntakeArmState intakeArmState) {
        return startEnd(() -> this.intakeArmState = intakeArmState, () -> {});
    }


    @Override
    public void periodic() {
        intakeMotor.set(intakeMotorState.power);
        intakeArm.setControl(motionMagicVoltage.withPosition(intakeArmState.pos));
    }
}
