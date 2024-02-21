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
  private double lastSetSpeed = 0.0;

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

    SendableRegistry.addLW(this, "PIDController", Constants.DriveConstants.SHOOTER_MOTOR_PORT);
    SendableRegistry.setName(this, "ShooterSubsystem");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
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
          lastSetSpeed = speed;
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
          lastSetSpeed = Constants.ShooterConstants.REVERSE_PERCENT;
          m_shooterMotor.set(lastSetSpeed);
          m_shooterMotor2.set(lastSetSpeed);
        },
        this);
  }

  public Command stopMotors() {
    return new WaitUntilShooterDoneCommand(
        () -> {
          lastSetSpeed = 0.0;
          m_shooterMotor
              .getPIDController()
              .setReference(lastSetSpeed, CANSparkBase.ControlType.kVelocity);
          m_shooterMotor2
              .getPIDController()
              .setReference(lastSetSpeed, CANSparkBase.ControlType.kVelocity);
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

    return Math.abs(currentSpeed1 - lastSetSpeed) < tolerance
        && Math.abs(currentSpeed2 - lastSetSpeed) < tolerance;
  }

  @Override
  public void periodic() {
    m_shooterCurrentVoltage.log(m_shooterMotor.getBusVoltage());
    m_shooterCurrentVelocity.log(m_shooterMotor.getEncoder().getVelocity());
    m_shooterDesiredOutput.log(lastSetSpeed);
  }
}

class WaitUntilShooterDoneCommand extends Command {
  private final Runnable action;
  private final ShooterSubsystem shooterSubsystem;

  public WaitUntilShooterDoneCommand(Runnable action, ShooterSubsystem shooterSubsystem) {
    super();
    this.action = action;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void initialize() {
    action.run();
  }

  @Override
  public boolean isFinished() {
    double tolerance = Constants.ShooterConstants.SHOOTER_TOLERANCE;
    return shooterSubsystem.isAtSetSpeed(tolerance);
  }
}
