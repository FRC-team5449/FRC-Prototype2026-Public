package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final Compressor compressor;
    private final Solenoid solenoid;
    private boolean extended = false;

    public Climber() {
        compressor = new Compressor(
                ClimberConstants.kPneumaticHubId,
                ClimberConstants.kModuleType
        );

        solenoid = new Solenoid(
                ClimberConstants.kPneumaticHubId,
                ClimberConstants.kModuleType,
                ClimberConstants.kSolenoidChannel
        );
    }

    public void extend() {
        solenoid.set(true);
        extended = true;
    }

    public void retract() {
        solenoid.set(false);
        extended = false;
    }

    public void toggle() {
        if (extended) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isExtended() {
        return extended;
    }

    public boolean isCompressorRunning() {
        return compressor.isEnabled();
    }

    public double getCompressorCurrentAmps() {
        return compressor.getCurrent();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public Command toggleCommand() {
        return runOnce(this::toggle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber/Extended", extended);
        SmartDashboard.putBoolean("Climber/Compressor Running", compressor.isEnabled());
        SmartDashboard.putNumber("Climber/Compressor Current (A)", compressor.getCurrent());
    }
}
