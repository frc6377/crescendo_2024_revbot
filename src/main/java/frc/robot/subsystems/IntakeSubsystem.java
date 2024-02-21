package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.DebugEntry;

public class IntakeSubsystem extends SubsystemBase {
  public final CANSparkMax m_intakeMotor;
  public final CANSparkMax m_intakeMotor2;
  public double m_intakeLastSetSpeed = 0.0;
  public double m_intake2LastSetSpeed = 0.0;
  private DebugEntry<Double> m_intakeCurrentVoltage;
  private DebugEntry<Double> m_intake2CurrentVoltage;
  private DebugEntry<Double> m_intakeCurrentVelocity;
  private DebugEntry<Double> m_intake2CurrentVelocity;
  private DebugEntry<Double> m_intakeDesiredOutput;
  private DebugEntry<Double> m_intake2DesiredOutput;

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

    SendableRegistry.addLW(this, "PIDController", Constants.DriveConstants.kintakeMotorPort);
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

  public Command runIntake() {
    return new WaitUntilIntakeDoneCommand(
        () -> {
          m_intakeLastSetSpeed = Constants.IntakeConstants.firstMotorVelocity;
          m_intake2LastSetSpeed = Constants.IntakeConstants.secondMotorVelocity;
          m_intakeMotor
              .getPIDController()
              .setReference(m_intakeLastSetSpeed, CANSparkBase.ControlType.kVelocity);
          m_intakeMotor2
              .getPIDController()
              .setReference(m_intake2LastSetSpeed, CANSparkBase.ControlType.kVelocity);
        },
        this);
  }

  public Command reverseIntake() {
    return new InstantCommand(
        () -> {
          m_intakeMotor.set(-Constants.IntakeConstants.firstMotorVelocity);
          m_intakeMotor2.set(-Constants.IntakeConstants.secondMotorVelocity);
        },
        this);
  }

  public Command stopMotors() {
    return new InstantCommand(
        () -> {
          m_intakeLastSetSpeed = 0.0;
          m_intake2LastSetSpeed = 0.0;
          m_intakeMotor.set(m_intakeLastSetSpeed);
          m_intakeMotor2.set(m_intake2LastSetSpeed);
        },
        this);
  }

  public boolean isAtSetSpeed(double tolerance) {
    double intakeSpeedDiff =
        Math.abs(m_intakeMotor.getEncoder().getVelocity() - m_intakeLastSetSpeed);
    double intake2SpeedDiff =
        Math.abs(m_intakeMotor2.getEncoder().getVelocity() - m_intake2LastSetSpeed);
    return intakeSpeedDiff <= tolerance && intake2SpeedDiff <= tolerance;
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

class WaitUntilIntakeDoneCommand extends Command {
  private final Runnable action;
  private final IntakeSubsystem intakeSubsystem;

  public WaitUntilIntakeDoneCommand(Runnable action, IntakeSubsystem intakeSubsystem) {
    super();
    this.action = action;
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void initialize() {
    action.run();
  }

  @Override
  public boolean isFinished() {
    double tolerance = Constants.IntakeConstants.INTAKE_TOLERANCE;
    return intakeSubsystem.isAtSetSpeed(tolerance);
  }
}
