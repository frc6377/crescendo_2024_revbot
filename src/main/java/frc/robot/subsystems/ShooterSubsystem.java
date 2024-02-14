package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_shooterMotor;
  private final CANSparkMax m_shooterMotor2;
  private double lastSetSpeed = 0.0;

  public ShooterSubsystem() {
    SendableRegistry.addLW(this, "ShooterSubsystem");
    m_shooterMotor =
        new CANSparkMax(Constants.DriveConstants.kShooterMotorPort, MotorType.kBrushless);
    m_shooterMotor2 =
        new CANSparkMax(Constants.DriveConstants.kShooterMotorPort2, MotorType.kBrushless);

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
  }

  public void setShooterSpeed(double speed) {
    lastSetSpeed = speed;
    m_shooterMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
    m_shooterMotor2.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  public void reverseShooter() {
    lastSetSpeed = Constants.ShooterConstants.REVERSE_PERCENT;
    m_shooterMotor.set(lastSetSpeed);
    m_shooterMotor2.set(lastSetSpeed);
  }

  public void stopMotors() {
    lastSetSpeed = 0.0;
    m_shooterMotor
        .getPIDController()
        .setReference(lastSetSpeed, CANSparkBase.ControlType.kVelocity);
    m_shooterMotor2
        .getPIDController()
        .setReference(lastSetSpeed, CANSparkBase.ControlType.kVelocity);
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

  public boolean isAtSetSpeed() {
    double currentSpeed1 = m_shooterMotor.getEncoder().getVelocity();
    double currentSpeed2 = m_shooterMotor2.getEncoder().getVelocity();

    return Math.abs(currentSpeed1 - lastSetSpeed) < Constants.ShooterConstants.SHOOTER_ISDONE_SPEED
        && Math.abs(currentSpeed2 - lastSetSpeed) < Constants.ShooterConstants.SHOOTER_ISDONE_SPEED;
  }
}
