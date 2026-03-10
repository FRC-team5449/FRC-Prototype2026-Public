package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public Hood() {
        motor = new SparkMax(HoodConstants.kMotorCanId, MotorType.kBrushless);
        motor.configure(
            new SparkMaxConfig().inverted(false).idleMode(IdleMode.kBrake),
            ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        );
        encoder = motor.getEncoder();
    }

    public void set(double power) {
        motor.set(power);
    }

    public void stop() {
        motor.set(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/Position", encoder.getPosition());
        SmartDashboard.putNumber("Hood/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Hood/Current", motor.getOutputCurrent());
    }
}
