package frc.robot.subsystems.hood;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public enum Position {
        DOWN(HoodConstants.kMinPosition),
        MIDDLE(HoodConstants.kMidPosition),
        UP(HoodConstants.kMaxPosition);

        public final double value;
        Position(double value) { this.value = value; }
    }

    private double targetPosition = 0.0;
    private boolean closedLoop = false;

    public Hood() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake);
        config.closedLoop
            .pid(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD)
            .outputRange(-0.5, 0.5);

        motor = new SparkMax(HoodConstants.kMotorCanId, MotorType.kBrushless);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        encoder = motor.getEncoder();
    }

    public void setPosition(Position pos) {
        setPosition(pos.value);
    }

    public void setPosition(double position) {
        targetPosition = MathUtil.clamp(position, HoodConstants.kMinPosition, HoodConstants.kMaxPosition);
        closedLoop = true;
    }

    public void set(double power) {
        closedLoop = false;
        motor.set(power);
    }

    public void stop() {
        closedLoop = false;
        motor.set(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public boolean atTarget() {
        return closedLoop && Math.abs(encoder.getPosition() - targetPosition) < 0.3;
    }

    @Override
    public void periodic() {
        if (closedLoop) {
            motor.getClosedLoopController().setReference(
                targetPosition,
                SparkMax.ControlType.kPosition
            );
        }

        SmartDashboard.putNumber("Hood/Position", encoder.getPosition());
        SmartDashboard.putNumber("Hood/Target", targetPosition);
        SmartDashboard.putNumber("Hood/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Hood/Current", motor.getOutputCurrent());
        SmartDashboard.putBoolean("Hood/AtTarget", atTarget());
    }
}
