package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor;
    private PositionTorqueCurrentFOC mTorqueCurrentConfigs = new PositionTorqueCurrentFOC(0);

    public Turret() {
        turretMotor = new TalonFX(TurretConstants.turretMotorCanId, TurretConstants.turretCanBus);
    }

    public void setAngle(Rotation2d angleRotations) {
        double clamped =
            MathUtil.clamp(
                    angleRotations.getRadians(),
                    TurretConstants.MIN_ANGLE.getRadians(),
                    TurretConstants.MAX_ANGLE.getRadians()
            );
        turretMotor.setControl(mTorqueCurrentConfigs.withPosition(angleRotations.getRotations()));
    }

    @Override
    public void periodic() {
    }
}
