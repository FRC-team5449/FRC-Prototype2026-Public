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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor, leftIntakeArm, rightIntakeArm;
    private final MotionMagicVoltage motionMagicVoltage;
    @AutoLogOutput private Goal goal = Goal.RETRACT;

    private static final LoggedTunableNumber armIntakePos = new LoggedTunableNumber("Intake/Intake/ArmPos", 12.0);
    private static final LoggedTunableNumber armRetractPos = new LoggedTunableNumber("Intake/Retract/ArmPos", 12.0);
    private static final LoggedTunableNumber rollerIntakeVolts = new LoggedTunableNumber("Intake/Intake/ArmPos", 12.0);
    private static final LoggedTunableNumber rollerOuttakeVolts = new LoggedTunableNumber("Intake/Intake/ArmPos", -8.0);
    private static final LoggedTunableNumber rollerStopVolts = new LoggedTunableNumber("Intake/Intake/ArmPos", 0);

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorCanId, IntakeConstants.intakeCanBus);
        leftIntakeArm = new TalonFX(IntakeConstants.leftIntakeArmCanId, IntakeConstants.intakeCanBus);
        rightIntakeArm = new TalonFX(IntakeConstants.rightIntakeArmCanId, IntakeConstants.intakeCanBus);
        motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

        this.goal = Goal.RETRACT;
    }

    public void setGoal(Goal goal) {
        if(this.goal == goal) return;
        this.goal = goal;
        //goaltimer.reset();
    }


    @Override
    public void periodic() {
        //calculate goal
        double rollerVolts = 0;
        switch(goal) {
            case RETRACT -> {
                setArmGoal(armRetractPos.get());
                rollerVolts = rollerStopVolts.get();
            }
            case DEPLOY -> {
                setArmGoal(armIntakePos.get());
                rollerVolts = rollerStopVolts.get();
            }
            case INTAKE -> {
                setArmGoal(armIntakePos.get());
                rollerVolts = rollerIntakeVolts.get();
            }
            case OUTTAKE -> {
                setArmGoal(armIntakePos.get());
                rollerVolts = rollerOuttakeVolts.get();
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
    OUTTAKE
  }
}
