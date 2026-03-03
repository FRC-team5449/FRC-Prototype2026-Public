package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
    public static CANBus intakeCanBus = new CANBus("rio");
    public static int intakeMotorCanId = 15;
    public static int intakeArmCanId = 16;

    public static TalonFXConfiguration config = new TalonFXConfiguration();

    public static TalonFXConfiguration getConfigs() {
        config.Slot0.kP = 0.001;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0001;
        config.Slot0.kV = 0.12;  // 前馈，可选
        config.MotionMagic.MotionMagicCruiseVelocity = 80;   // rps
        config.MotionMagic.MotionMagicAcceleration = 160;     // rps/s
        config.MotionMagic.MotionMagicJerk = 1600;            // rps/s/s，可选

        return config;
    }
}
