package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;
    private VelocityVoltage velocityVoltage;
    private Goal goal;
    private double rpsSetpoint;

    public Shooter() {
        leftShooterMotor = new TalonFX(ShooterConstants.leftShooterMotorCanId, ShooterConstants.shooterCanBus);
        rightShooterMotor = new TalonFX(ShooterConstants.rightShooterMotorCanId, ShooterConstants.shooterCanBus);

        velocityVoltage = new VelocityVoltage(0).withSlot(0);

        goal = Goal.STOP;
    }

    public enum Goal {
        HUB,
        ALLIANCE,
        STOP;
    }

    public void setTarget(double rpm) {
        rpsSetpoint = rpm;
    }

    private void setRPS(double rpm) {
        leftShooterMotor.setControl(velocityVoltage.withVelocity(rpm));
        rightShooterMotor.setControl(new Follower(ShooterConstants.leftShooterMotorCanId, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        switch(goal) {
            case STOP -> {
                setRPS(0);
            }
            case HUB -> {
                setRPS(rpsSetpoint);
            }
            case ALLIANCE -> {
                setRPS(rpsSetpoint);
            }
        }
    }
}
