package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
    public static CANBus turretCanBus = new CANBus("rio");
    public static int turretMotorCanId = 24;

    public static Rotation2d MIN_ANGLE = new Rotation2d(Math.toRadians(-185));
    public static Rotation2d MAX_ANGLE = new Rotation2d(Math.toRadians(185));
    public static final double gearRatio = 43.0555555555;

    public static TalonFXConfiguration config = new TalonFXConfiguration();

    public static TalonFXConfiguration getConfigs() {
        config.Slot0.kP = 12.5;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.2;
        config.Slot0.kS = 20;
        // config.Voltage.PeakForwardVoltage = 3;
        // config.Voltage.PeakReverseVoltage = -3;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        // config.MotionMagic.MotionMagicCruiseVelocity = 80;   // rps
        // config.MotionMagic.MotionMagicAcceleration = 160;     // rps/s 
        // config.MotionMagic.MotionMagicJerk = 1600;            // rps/s/s, optional

        return config;
    }
}
