package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTunableNumber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor, leftIntakeArm, rightIntakeArm;
    private final MotionMagicVoltage motionMagicVoltage;
    @AutoLogOutput private Goal goal = Goal.RETRACT;

    

    private static final LoggedTunableNumber armIntakePos = new LoggedTunableNumber("Intake/Intake/ArmPos", 19.4423);
    private static final LoggedTunableNumber armRetractPos = new LoggedTunableNumber("Intake/Retract/ArmPos", -0.210449);
    private static final LoggedTunableNumber armMiddlePos = new LoggedTunableNumber("Intake/Middle/ArmPos", 9.0);
    private static final LoggedTunableNumber rollerIntakeVolts = new LoggedTunableNumber("Intake/Intake/RollerVolts", 12.0);
    private static final LoggedTunableNumber rollerOuttakeVolts = new LoggedTunableNumber("Intake/Outtake/RollerVolts", -8.0);
    private static final LoggedTunableNumber rollerStopVolts = new LoggedTunableNumber("Intake/Stop/RollerVolts", 0);

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorCanId, new CANBus("canivore"));
        leftIntakeArm = new TalonFX(IntakeConstants.leftIntakeArmCanId, IntakeConstants.intakeCanBus);
        rightIntakeArm = new TalonFX(IntakeConstants.rightIntakeArmCanId, IntakeConstants.intakeCanBus);
        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

        leftIntakeArm.getConfigurator().apply(IntakeConstants.getConfigs());
        rightIntakeArm.getConfigurator().apply(IntakeConstants.getConfigs());

        this.goal = Goal.DEPLOY;    
    }

    public Goal getGoal() {
        return this.goal;
    }

    public void setGoal(Goal goal) {
        if(this.goal == goal) return;
        this.goal = goal;
    }


    @Override
    public void periodic() {
        //calculate goal
        double rollerVolts = 0;
        switch(goal) {
            case RETRACT -> {
                // setArmGoal(armRetractPos.get());
                rollerVolts = rollerStopVolts.get();
            }
            case DEPLOY -> {
                // setArmGoal(armIntakePos.get());
                rollerVolts = rollerStopVolts.get();
            }
            case INTAKE -> {
                // setArmGoal(armIntakePos.get());
                rollerVolts = rollerIntakeVolts.get();
            }
            case OUTTAKE -> {
                // setArmGoal(armIntakePos.get());
                rollerVolts = rollerOuttakeVolts.get();
            }
            case MIDDLE -> {
                // setArmGoal(armMiddlePos.get());
                rollerVolts = rollerIntakeVolts.get();
            }
        }
        
        intakeMotor.setControl(new VoltageOut(0.0).withOutput(rollerVolts));
        
    }

    private void setArmGoal(double pose) {
        leftIntakeArm.setControl(motionMagicVoltage.withPosition(pose));
        rightIntakeArm.setControl(new Follower(IntakeConstants.leftIntakeArmCanId, MotorAlignmentValue.Opposed));
    }

    public enum Goal {
    RETRACT,
    DEPLOY,
    INTAKE,
    OUTTAKE,
    MIDDLE
  }
}
