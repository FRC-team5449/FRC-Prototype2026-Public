package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretConstants {
    public static CANBus turretCanBus = new CANBus("canivore");
    public static int turretMotorCanId = 24;

    public static Rotation2d MIN_ANGLE = new Rotation2d(-45);
    public static Rotation2d MAX_ANGLE = new Rotation2d(45);
}
