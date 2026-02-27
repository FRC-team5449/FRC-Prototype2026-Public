package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

public class ShooterConstants {
    public static CANBus shooterCanBus = new CANBus("canivore");
    public static int leftShooterMotorCanId = 20;
    public static int rightShooterMotorCanId = 21;
}
