package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;
    private Goal goal;
    private double turretSetpoint;
    private PositionVoltage mPositionVoltage = new PositionVoltage(0).withSlot(0);

    public Turret() {
        turretMotor = new TalonFX(TurretConstants.turretMotorCanId, TurretConstants.turretCanBus);

        turretMotor.getConfigurator().apply(TurretConstants.getConfigs());
        setGoal(Goal.STOP);
    }

    public void setAngle(Rotation2d angleRotations) {
        double clamped =
            MathUtil.clamp(
                    angleRotations.getRadians(),
                    TurretConstants.MIN_ANGLE.getRadians(),
                    TurretConstants.MAX_ANGLE.getRadians()
            );
        turretSetpoint = clamped / (2 * Math.PI);
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
        switch (goal) {
            case STOP -> {
                turretMotor.setControl(mPositionVoltage.withPosition(0));
            }
            case HUB -> {
                turretMotor.setControl(mPositionVoltage.withPosition(turretSetpoint));
            }
            case ALLIANCE -> {
                turretMotor.setControl(mPositionVoltage.withPosition(turretSetpoint));
            }
        }
        Logger.recordOutput("turretSetpoint", turretSetpoint);
    }

    public enum Goal {
        STOP,
        HUB,
        ALLIANCE;
    }
}
