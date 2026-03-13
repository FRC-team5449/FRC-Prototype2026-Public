package frc.robot.subsystems.index;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
    private final TalonFX indexMotor;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public enum IndexState {
        ACTIVE(0.8),
        STOP(0.0),
        REVERSE(-0.3);

        public double power;

        IndexState(double power) {
            this.power = power;
        }
    }

    private IndexState indexState;
    private IndexState requestedState;

    private double stallStartTime = -1;
    private boolean isStalled = false;

    public Index() {
        indexMotor = new TalonFX(IndexConstants.indexMotorCanId, "rio");
        indexState = IndexState.STOP;
        requestedState = IndexState.STOP;
    }

    public void setIndexState(IndexState indexState) {
        this.requestedState = indexState;
    }

    public IndexState getIndexState() {
        return indexState;
    }

    public boolean isStalled() {
        return isStalled;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double statorCurrent = indexMotor.getStatorCurrent().getValueAsDouble();
        Logger.recordOutput("indexMotorCurrent", statorCurrent);

        if (isStalled) {
            if (now - stallStartTime >= IndexConstants.stallCooldownTime) {
                isStalled = false;
                stallStartTime = -1;
                indexState = requestedState;
            } else {
                indexState = IndexState.REVERSE;
            }
        } else {
            indexState = requestedState;

            if (indexState == IndexState.ACTIVE
                    && statorCurrent > IndexConstants.stallCurrentThreshold) {
                isStalled = true;
                stallStartTime = now;
                indexState = IndexState.REVERSE;
            }
        }

        indexMotor.setControl(dutyCycleOut.withOutput(indexState.power));
    }
}
