package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterConstants {
    public static CANBus shooterCanBus = new CANBus("rio");
    public static int leftShooterMotorCanId = 23;
    public static int rightShooterMotorCanId = 22;

    public static TalonFXConfiguration config = new TalonFXConfiguration();

    public static TalonFXConfiguration getConfigs() {
        config.Slot0.kP = 0.4;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.01;
        config.Slot0.kV = 0.12;
        config.Slot0.kS = 0.1;
        config.MotionMagic.MotionMagicCruiseVelocity = 80;   // rps
        config.MotionMagic.MotionMagicAcceleration = 160;     // rps/s 
        config.MotionMagic.MotionMagicJerk = 1600;            // rps/s/s，可选

        return config;
    }
}
