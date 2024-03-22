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

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_shooterMotor;
  private final CANSparkMax m_shooterMotor2;
  private DebugEntry<Double> m_shooterCurrentVoltage;
  private DebugEntry<Double> m_shooterCurrentVelocity;
  private DebugEntry<Double> m_shooterDesiredOutput;
  private double m_shooterLastSetSpeed = 0.0;
  public double REVERSE_PERCENT = Constants.ShooterConstants.REVERSE_PERCENT;
  public double SHOOTER_RPM = Constants.ShooterConstants.SHOOTER_RPM;

  public ShooterSubsystem() {
    m_shooterMotor =
        new CANSparkMax(Constants.DriveConstants.SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    m_shooterMotor2 =
        new CANSparkMax(Constants.DriveConstants.SHOOTER_MOTOR_PORT_2, MotorType.kBrushless);

    m_shooterMotor.restoreFactoryDefaults();
    m_shooterMotor.setSmartCurrentLimit(40);
    m_shooterMotor2.restoreFactoryDefaults();
    m_shooterMotor2.setSmartCurrentLimit(40);

    m_shooterMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    m_shooterMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    m_shooterMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    m_shooterMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);
    m_shooterMotor2.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    m_shooterMotor2.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    m_shooterMotor2.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    m_shooterMotor2.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);

    SendableRegistry.addLW(this, "ShooterSubsystem");
    m_shooterCurrentVoltage = new DebugEntry<Double>(0.0, "shooterMotor1_vo", this);
    m_shooterCurrentVelocity = new DebugEntry<Double>(0.0, "shooterMotor2_ve", this);
    m_shooterDesiredOutput = new DebugEntry<Double>(0.0, "shooterMotor1_de", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty(
        "REVERSE_PERCENT", () -> REVERSE_PERCENT, value -> REVERSE_PERCENT = value);
    builder.addDoubleProperty("SHOOTER_RPM", () -> SHOOTER_RPM, value -> SHOOTER_RPM = value);
    builder.addDoubleProperty(
        "p",
        () -> m_shooterMotor.getPIDController().getP(),
        (p) -> {
          m_shooterMotor.getPIDController().setP(p);
          m_shooterMotor2.getPIDController().setP(p);
        });
    builder.addDoubleProperty(
        "i",
        () -> m_shooterMotor.getPIDController().getI(),
        (i) -> {
          m_shooterMotor.getPIDController().setI(i);
          m_shooterMotor2.getPIDController().setI(i);
        });
    builder.addDoubleProperty(
        "d",
        () -> m_shooterMotor.getPIDController().getD(),
        (d) -> {
          m_shooterMotor.getPIDController().setD(d);
          m_shooterMotor2.getPIDController().setD(d);
        });
    builder.addDoubleProperty(
        "ff",
        () -> m_shooterMotor.getPIDController().getFF(),
        (ff) -> {
          m_shooterMotor.getPIDController().setFF(ff);
          m_shooterMotor2.getPIDController().setFF(ff);
        });
  }

  public Command setShooterSpeedCommand(double speed) {
    return new WaitUntilShooterDoneCommand(
        () -> {
          m_shooterLastSetSpeed = speed;
          m_shooterMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
          m_shooterMotor2
              .getPIDController()
              .setReference(speed, CANSparkBase.ControlType.kVelocity);
        },
        this);
  }

  public Command reverseShooterCommand() {
    return new InstantCommand(
        () -> {
          m_shooterLastSetSpeed = REVERSE_PERCENT;
          m_shooterMotor.set(m_shooterLastSetSpeed);
          m_shooterMotor2.set(m_shooterLastSetSpeed);
        },
        this);
  }

  public Command stopMotors() {
    return new WaitUntilShooterDoneCommand(
        () -> {
          m_shooterLastSetSpeed = 0.0;
          m_shooterMotor
              .getPIDController()
              .setReference(m_shooterLastSetSpeed, CANSparkBase.ControlType.kVelocity);
          m_shooterMotor2
              .getPIDController()
              .setReference(m_shooterLastSetSpeed, CANSparkBase.ControlType.kVelocity);
        },
        this);
  }

  public double getSpeeds() {
    if (m_shooterMotor.getEncoder().getVelocity() != m_shooterMotor2.getEncoder().getVelocity()) {
      return 0;
    } else {
      return m_shooterMotor.getEncoder().getVelocity();
    }
  }

  public void setCoastMode() {
    m_shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setBrakeMode() {
    m_shooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public boolean isAtSetSpeed(double tolerance) {
    double currentSpeed1 = m_shooterMotor.getEncoder().getVelocity();
    double currentSpeed2 = m_shooterMotor2.getEncoder().getVelocity();

    return Math.abs(currentSpeed1 - m_shooterLastSetSpeed) < tolerance
        && Math.abs(currentSpeed2 - m_shooterLastSetSpeed) < tolerance;
  }

  @Override
  public void periodic() {
    m_shooterCurrentVoltage.log(m_shooterMotor.getBusVoltage());
    m_shooterCurrentVelocity.log(m_shooterMotor.getEncoder().getVelocity());
    m_shooterDesiredOutput.log(m_shooterLastSetSpeed);
  }
}

class WaitUntilShooterDoneCommand extends Command {
  private final Runnable action;
  private final ShooterSubsystem shooterSubsystem;
  public double SHOOTER_TOLERANCE = Constants.ShooterConstants.SHOOTER_TOLERANCE;

  public WaitUntilShooterDoneCommand(Runnable action, ShooterSubsystem shooterSubsystem) {
    super();
    this.action = action;
    this.shooterSubsystem = shooterSubsystem;
    SendableRegistry.addLW(this, "WaitUntilShooterDoneCommand", "shooterTolerance");
  }

  @Override
  public void initialize() {
    action.run();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "shooterTolerance", () -> SHOOTER_TOLERANCE, value -> SHOOTER_TOLERANCE = value);
  }

  @Override
  public boolean isFinished() {
    double tolerance = SHOOTER_TOLERANCE;
    return shooterSubsystem.isAtSetSpeed(tolerance);
  }
}
