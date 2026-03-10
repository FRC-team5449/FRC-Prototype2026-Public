package frc.robot.subsystems.index;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;

public class Index extends SubsystemBase {
    private final TalonFX indexMotor;

    public enum IndexState {
        ACTIVE(-0.65),
        STOP(0.0);

        public double power;

        IndexState(double power) {
            this.power = power;
        }
    }

    private IndexState indexState;

    public Index() {
        indexMotor = new TalonFX(IndexConstants.indexMotorCanId, "rio");
        indexState = IndexState.STOP;
    }

    public void setIndexState(IndexState indexState) {
        this.indexState = indexState;
    }

    public IndexState getIndexState() {
        return indexState;
    }

    @Override
    public void periodic() {
        indexMotor.setControl(new DutyCycleOut(indexState.power));
    }
}
