package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final CANSparkMax shooterMotor2;
  private double lastSetSpeed = 0.0;

  public ShooterSubsystem() {
    SendableRegistry.addLW(this, "ShooterSubsystem");
    shooterMotor =
        new CANSparkMax(Constants.DriveConstants.kShooterMotorPort2, MotorType.kBrushless);
    shooterMotor2 =
        new CANSparkMax(Constants.DriveConstants.kShooterMotorPort2, MotorType.kBrushless);

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setSmartCurrentLimit(40);
    shooterMotor2.restoreFactoryDefaults();
    shooterMotor2.setSmartCurrentLimit(40);

    shooterMotor.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    shooterMotor.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    shooterMotor.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    shooterMotor.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);
    shooterMotor2.getPIDController().setP(Constants.ShooterConstants.SHOOTER_P);
    shooterMotor2.getPIDController().setI(Constants.ShooterConstants.SHOOTER_I);
    shooterMotor2.getPIDController().setD(Constants.ShooterConstants.SHOOTER_D);
    shooterMotor2.getPIDController().setFF(Constants.ShooterConstants.SHOOTER_FF);

    setIdleSpeed();
  }

  public void setShooterSpeed(double speed) {
    lastSetSpeed = speed;
    shooterMotor.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
    shooterMotor2.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
  }

  public double getSpeeds() {
    if (shooterMotor.getEncoder().getVelocity() != shooterMotor2.getEncoder().getVelocity()) {
      return 0;
    } else {
      return shooterMotor.getEncoder().getVelocity();
    }
  }

  public void setCoastMode() {
    shooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void setBrakeMode() {
    shooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shooterMotor2.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setIdleSpeed() {
    shooterMotor.set(-0.1);
    shooterMotor2.set(-0.1);
  }

  public boolean isAtSetSpeed() {
    double currentSpeed1 = shooterMotor.getEncoder().getVelocity();
    double currentSpeed2 = shooterMotor2.getEncoder().getVelocity();

    return Math.abs(currentSpeed1 - lastSetSpeed) < Constants.ShooterConstants.SHOOTER_ISDONE_SPEED
        && Math.abs(currentSpeed2 - lastSetSpeed) < Constants.ShooterConstants.SHOOTER_ISDONE_SPEED;
  }
}
