package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.DebugEntry;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_intakeMotor;
  private final CANSparkMax m_intakeMotor2;
  private DebugEntry<Double> m_intakeCurrentVoltage;
  private DebugEntry<Double> m_intake2CurrentVoltage;
  private DebugEntry<Double> m_intakeCurrentVelocity;
  private DebugEntry<Double> m_intake2CurrentVelocity;
  private DebugEntry<Double> m_intakeDesiredOutput;
  private DebugEntry<Double> m_intake2DesiredOutput;
  private double m_intakeLastSetSpeed = 0.0;
  private double m_intake2LastSetSpeed = 0.0;

  public IntakeSubsystem() {
    m_intakeMotor =
        new CANSparkMax(Constants.DriveConstants.kintakeMotorPort, MotorType.kBrushless);
    m_intakeMotor2 =
        new CANSparkMax(Constants.DriveConstants.kintakeMotorPort2, MotorType.kBrushless);

    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setSmartCurrentLimit(40);
    m_intakeMotor2.restoreFactoryDefaults();
    m_intakeMotor2.setSmartCurrentLimit(40);

    m_intakeMotor.getPIDController().setP(Constants.IntakeConstants.INTAKE_P);
    m_intakeMotor.getPIDController().setI(Constants.IntakeConstants.INTAKE_I);
    m_intakeMotor.getPIDController().setD(Constants.IntakeConstants.INTAKE_D);
    m_intakeMotor.getPIDController().setFF(Constants.IntakeConstants.INTAKE_FF);
    m_intakeMotor2.getPIDController().setP(Constants.IntakeConstants.INTAKE_P);
    m_intakeMotor2.getPIDController().setI(Constants.IntakeConstants.INTAKE_I);
    m_intakeMotor2.getPIDController().setD(Constants.IntakeConstants.INTAKE_D);
    m_intakeMotor2.getPIDController().setFF(Constants.IntakeConstants.INTAKE_FF);

    SendableRegistry.addLW(this, "PIDController", Constants.DriveConstants.kShooterMotorPort);
    SendableRegistry.setName(this, "IntakeSubsystem");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty(
        "p",
        () -> m_intakeMotor.getPIDController().getP(),
        (p) -> {
          m_intakeMotor.getPIDController().setP(p);
          m_intakeMotor2.getPIDController().setP(p);
        });
    builder.addDoubleProperty(
        "i",
        () -> m_intakeMotor.getPIDController().getI(),
        (i) -> {
          m_intakeMotor.getPIDController().setI(i);
          m_intakeMotor2.getPIDController().setI(i);
        });
    builder.addDoubleProperty(
        "d",
        () -> m_intakeMotor.getPIDController().getD(),
        (d) -> {
          m_intakeMotor.getPIDController().setD(d);
          m_intakeMotor2.getPIDController().setD(d);
        });
    builder.addDoubleProperty(
        "ff",
        () -> m_intakeMotor.getPIDController().getFF(),
        (ff) -> {
          m_intakeMotor.getPIDController().setFF(ff);
          m_intakeMotor2.getPIDController().setFF(ff);
        });
  }

  public void runIntake() {
    m_intakeLastSetSpeed = Constants.IntakeConstants.firstMotorVelocity;
    m_intake2LastSetSpeed = Constants.IntakeConstants.secondMotorVelocity;
    m_intakeMotor
        .getPIDController()
        .setReference(m_intakeLastSetSpeed, CANSparkBase.ControlType.kVelocity);
    m_intakeMotor2
        .getPIDController()
        .setReference(m_intake2LastSetSpeed, CANSparkBase.ControlType.kVelocity);
  }

  public void reverseIntake() {
    m_intakeLastSetSpeed = -Constants.IntakeConstants.firstMotorVelocity;
    m_intake2LastSetSpeed = -Constants.IntakeConstants.secondMotorVelocity;
    m_intakeMotor.set(m_intakeLastSetSpeed);
    m_intakeMotor2.set(m_intake2LastSetSpeed);
  }

  public void stopMotors() {
    m_intakeLastSetSpeed = 0.0;
    m_intake2LastSetSpeed = 0.0;
    m_intakeMotor.set(m_intakeLastSetSpeed);
    m_intakeMotor2.set(m_intake2LastSetSpeed);
  }

  @Override
  public void periodic() {
    m_intakeCurrentVoltage.log(m_intakeMotor.getBusVoltage());
    m_intake2CurrentVoltage.log(m_intakeMotor2.getBusVoltage());
    m_intakeCurrentVelocity.log(m_intakeMotor.getEncoder().getVelocity());
    m_intake2CurrentVelocity.log(m_intakeMotor2.getEncoder().getVelocity());
    m_intakeDesiredOutput.log(m_intakeLastSetSpeed);
    m_intake2DesiredOutput.log(m_intake2LastSetSpeed);
  }
}
