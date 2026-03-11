package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final TalonFX leftShooterMotor;
    private final TalonFX rightShooterMotor;
    private VelocityVoltage velocityVoltage;
    private Goal goal;
    private double rpmSetpoint;

    private double openLoopPower;

    public final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", ShooterConstants.getConfigs().Slot0.kP);
    public final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", ShooterConstants.getConfigs().Slot0.kI);
    public final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", ShooterConstants.getConfigs().Slot0.kD);
    public final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", ShooterConstants.getConfigs().Slot0.kV);

    public Shooter() {
        leftShooterMotor = new TalonFX(ShooterConstants.leftShooterMotorCanId, ShooterConstants.shooterCanBus);
        rightShooterMotor = new TalonFX(ShooterConstants.rightShooterMotorCanId, ShooterConstants.shooterCanBus);

        velocityVoltage = new VelocityVoltage(0).withSlot(0);
        leftShooterMotor.getConfigurator().apply(ShooterConstants.getConfigs());
        rightShooterMotor.getConfigurator().apply(ShooterConstants.getConfigs());
        goal = Goal.HUB;

        openLoopPower = 0;
    }

    public enum Goal {
        HUB,
        ALLIANCE,
        STOP,
        OPENLOOP;
    }

    public void setTarget(double rpm) {
        rpmSetpoint = rpm;
    }

    public void setOpenLoopPower(double power) {
        this.openLoopPower = power;
    }

    private void setRPM(double rpm) {
        leftShooterMotor.setControl(velocityVoltage.withVelocity(rpm));
        rightShooterMotor.setControl(new Follower(ShooterConstants.leftShooterMotorCanId, MotorAlignmentValue.Opposed));
    }

    public void setGoal(Goal targetGoal) {
        this.goal = targetGoal;
    }

    @Override
    public void periodic() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (values) -> {
                Slot0Configs slot0 = new Slot0Configs();
                slot0.kP = kP.get();
                slot0.kI = kI.get();
                slot0.kD = kD.get();
                slot0.kV = kV.get();

                leftShooterMotor.getConfigurator().apply(slot0);
                rightShooterMotor.getConfigurator().apply(slot0);
            },
            kP, kI, kD, kV
        );

        switch(goal) {
            case STOP -> {
                setRPM(0);
            }
            case HUB -> {
                setRPM(rpmSetpoint);
            }
            case ALLIANCE -> {
                setRPM(rpmSetpoint);
            }
            case OPENLOOP -> {
                leftShooterMotor.setControl(new DutyCycleOut(openLoopPower));
                rightShooterMotor.setControl(new Follower(ShooterConstants.leftShooterMotorCanId, MotorAlignmentValue.Opposed));
            }
        }

        Logger.recordOutput("Shooter/Velocity", leftShooterMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/Setpoint", rpmSetpoint);
    }
}
