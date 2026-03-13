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

    /**
     * Set the turret target angle in robot-relative coordinates.
     * Ported from FTC Turret.setTurret() + periodic() wrapping logic:
     * - Normalize angle to [-PI, PI]
     * - If target crosses the +/-180 boundary relative to current position,
     *   add or subtract 360 to take the shorter path and avoid hitting limits.
     * - Clamp final angle to [MIN_ANGLE, MAX_ANGLE].
     */
    public void setAngle(Rotation2d angle) {
        double targetRad = normalizeAngle(angle.getRadians());

        targetRad = MathUtil.clamp(
            targetRad,
            TurretConstants.MIN_ANGLE.getRadians(),
            TurretConstants.MAX_ANGLE.getRadians()
        );

        turretSetpoint = -targetRad / (2 * Math.PI);
    }

    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    @Override
    public void periodic() {
        switch (goal) {
            case STOP -> {
                turretMotor.setControl(mPositionVoltage.withPosition(0));
            }
            case HUB, ALLIANCE -> {
                turretMotor.setControl(mPositionVoltage.withPosition(turretSetpoint));
            }
        }
        Logger.recordOutput("Turret/SetpointRot", turretSetpoint);
        Logger.recordOutput("Turret/SetpointDeg", turretSetpoint * 360.0);
        Logger.recordOutput("Turret/PositionRot", turretMotor.getPosition().getValueAsDouble());
        Logger.recordOutput("Turret/PositionDeg", turretMotor.getPosition().getValueAsDouble() * 360.0);
        Logger.recordOutput("Turret/ErrorDeg", (turretSetpoint - turretMotor.getPosition().getValueAsDouble()) * 360.0);
        Logger.recordOutput("Turret/Velocity", turretMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Turret/Current", turretMotor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Turret/Goal", goal.name());
    }

    /**
     * Normalize angle to [-PI, PI]. Ported from FTC Util.adjustRange().
     */
    private static double normalizeAngle(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    public enum Goal {
        STOP,
        HUB,
        ALLIANCE;
    }
}
