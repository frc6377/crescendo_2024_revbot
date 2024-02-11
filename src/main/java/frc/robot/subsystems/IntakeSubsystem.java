package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final Spark m_intakeMotor = new Spark(DriveConstants.intakeMotor);
  private final Spark m_intakeMotor2 = new Spark(DriveConstants.intakeMotor2);

  public void runIntake() {
    m_intakeMotor.set(Constants.intakeConstants.firstMotorPercent);
    m_intakeMotor2.set(Constants.intakeConstants.secondMotorPercent);
  }

  public void reverseIntake() {
    m_intakeMotor.set(-Constants.intakeConstants.firstMotorPercent);
    m_intakeMotor2.set(-Constants.intakeConstants.secondMotorPercent);
  }

  public void stopMotors() {
    m_intakeMotor.set(0);
    m_intakeMotor2.set(0);
  }

  public Command runIntakeCommand() {
    return new StartEndCommand(this::runIntake, this::stopMotors, this).withName("Run Intake");
  }

  public Command reverseIntakeCommand() {
    return new StartEndCommand(this::reverseIntake, this::stopMotors, this)
        .withName("Reverse Intake");
  }
}
