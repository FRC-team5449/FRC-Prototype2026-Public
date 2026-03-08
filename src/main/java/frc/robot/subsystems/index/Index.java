package frc.robot.subsystems.index;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
    private final SparkMax leftIndexMotor;
    private final SparkMax rightIndexMotor;

    public enum IndexState {
        ACTIVE(-1.0),
        STOP(0.0);

        public double power;

        IndexState(double power) {
            this.power = power;
        }
    }

    private IndexState indexState;

    public Index() {
        leftIndexMotor = new SparkMax(IndexConstants.leftIndexMotorCanId, MotorType.kBrushless);
        rightIndexMotor = new SparkMax(IndexConstants.rightIndexMotorCanId, MotorType.kBrushless);
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
        leftIndexMotor.set(indexState.power);
        rightIndexMotor.set(indexState.power);
    }
}
