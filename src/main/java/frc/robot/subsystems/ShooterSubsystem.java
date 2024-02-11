package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

// Motor Configuration: Assume 2 motors for the shooter. They need to turn inwards towards each
// other in the right direction for the note to be pulled and and shot out from intake. The specific
// configuration can be figured out later when we can test.

// User Interaction: On a certain interaction (trigger), such as a controller button press, the
// shootCommand should start. This should only happen if there's a note and the right speed is
// reached.

// Logging: Log all actions performed by the motors for debugging and tracking purposes.

// Tolerance Function: Implement a tolerance function that returns true or false. This function will
// be used to determine whether the motors should shoot or not based on their current speed.

// Finish Condition: Implement the isFinished method to determine when the shooting action is
// done, might need to tie this in with intake to see if it is intaking.

// Idle Speed: This in't an actual idle speed, but the speed is negative so that the note doesn't
// get shot out prematurely, we will compose the intake and shooter for intaking, and shooting in
// a command factory.

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterMotor;
  private final CANSparkMax shooterMotor2;
  private double lastSetSpeed = 0.0;

  public ShooterSubsystem() {
    SendableRegistry.addLW(this, "ShooterSubsystem");
    shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort2, MotorType.kBrushless);
    shooterMotor2 = new CANSparkMax(ShooterConstants.kShooterMotorPort2, MotorType.kBrushless);

    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setSmartCurrentLimit(40);
    shooterMotor2.restoreFactoryDefaults();
    shooterMotor2.setSmartCurrentLimit(40);

    shooterMotor.getPIDController().setP(ShooterConstants.SHOOTER_P);
    shooterMotor.getPIDController().setI(ShooterConstants.SHOOTER_I);
    shooterMotor.getPIDController().setD(ShooterConstants.SHOOTER_D);
    shooterMotor.getPIDController().setFF(ShooterConstants.SHOOTER_FF);
    shooterMotor2.getPIDController().setP(ShooterConstants.SHOOTER_P);
    shooterMotor2.getPIDController().setI(ShooterConstants.SHOOTER_I);
    shooterMotor2.getPIDController().setD(ShooterConstants.SHOOTER_D);
    shooterMotor2.getPIDController().setFF(ShooterConstants.SHOOTER_FF);

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

    return Math.abs(currentSpeed1 - lastSetSpeed) < ShooterConstants.SHOOTER_ISDONE_SPEED
        && Math.abs(currentSpeed2 - lastSetSpeed) < ShooterConstants.SHOOTER_ISDONE_SPEED;
  }
}
