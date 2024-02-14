package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_intakeMotor2;

  public IntakeSubsystem() {
    m_intakeMotor =
        new CANSparkMax(Constants.DriveConstants.kintakeMotorPort, MotorType.kBrushless);
    m_intakeMotor2 =
        new CANSparkMax(Constants.DriveConstants.kintakeMotorPort2, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setSmartCurrentLimit(40);
    m_intakeMotor2.restoreFactoryDefaults();
    m_intakeMotor2.setSmartCurrentLimit(40);

    m_intakeMotor.getPIDController().setP(Constants.IntakeConstants.SHOOTER_P);
    m_intakeMotor.getPIDController().setI(Constants.IntakeConstants.SHOOTER_I);
    m_intakeMotor.getPIDController().setD(Constants.IntakeConstants.SHOOTER_D);
    m_intakeMotor.getPIDController().setFF(Constants.IntakeConstants.SHOOTER_FF);
    m_intakeMotor2.getPIDController().setP(Constants.IntakeConstants.SHOOTER_P);
    m_intakeMotor2.getPIDController().setI(Constants.IntakeConstants.SHOOTER_I);
    m_intakeMotor2.getPIDController().setD(Constants.IntakeConstants.SHOOTER_D);
    m_intakeMotor2.getPIDController().setFF(Constants.IntakeConstants.SHOOTER_FF);
  }

  public void runIntake() {
    m_intakeMotor
        .getPIDController()
        .setReference(
            Constants.IntakeConstants.firstMotorPercent, CANSparkBase.ControlType.kVelocity);
    m_intakeMotor2
        .getPIDController()
        .setReference(
            Constants.IntakeConstants.secondMotorPercent, CANSparkBase.ControlType.kVelocity);
  }

  public void reverseIntake() {
    m_intakeMotor.set(-Constants.IntakeConstants.firstMotorPercent);
    m_intakeMotor2.set(-Constants.IntakeConstants.secondMotorPercent);
  }

  public void stopMotors() {
    m_intakeMotor.set(0);
    m_intakeMotor2.set(0);
  }

  public Command reverseIntakeCommand() {
    return new StartEndCommand(this::reverseIntake, this::stopMotors, this)
        .withName("Reverse Intake");
  }
}
