package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;

    public Shooter() {
        leftShooterMotor = new TalonFX(ShooterConstants.leftShooterMotorCanId, ShooterConstants.shooterCanBus);
        rightShooterMotor = new TalonFX(ShooterConstants.rightShooterMotorCanId, ShooterConstants.shooterCanBus);
    }

    @Override
    public void periodic() {
        leftShooterMotor.set(-1);
        rightShooterMotor.set(-1);
    }
}
