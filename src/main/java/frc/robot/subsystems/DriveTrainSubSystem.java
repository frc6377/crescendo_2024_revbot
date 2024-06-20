package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.Supplier;

public class DriveTrainSubSystem extends SubsystemBase {
  public static final boolean kInvertLeftNotRight = false;
  private final VictorSPX m_leftMotor = new VictorSPX(DriveConstants.LEFT_MOTOR_PORT);
  private final VictorSPX m_leftMotor2 = new VictorSPX(DriveConstants.LEFT_MOTOR_PORT_2);
  private final VictorSPX m_rightMotor = new VictorSPX(DriveConstants.RIGHT_MOTOR_PORT);
  private final VictorSPX m_rightMotor2 = new VictorSPX(DriveConstants.RIGHT_MOTOR_PORT_2);
  private DifferentialDrive m_robotDrive;

  public DriveTrainSubSystem() {
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

    SendableRegistry.addLW(this, "DriveTrainSubSystem");

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    if (kInvertLeftNotRight) {
      m_leftMotor.setInverted(true);
    } else {
      m_rightMotor.setInverted(true);
    }

    m_robotDrive =
        new DifferentialDrive(
            speed -> m_leftMotor.set(ControlMode.PercentOutput, speed),
            speed -> m_rightMotor.set(ControlMode.PercentOutput, speed));
  }

  public void arcadeDrive(double fwd, double rot) {
    m_robotDrive.arcadeDrive(fwd, rot);
  }

  public void tankDrive(double lspeed, double rspeed) {
    m_robotDrive.tankDrive(lspeed, rspeed);
  }

  public void setSpeeds(double leftSpeed, double rightSpeed) {
    m_leftMotor.set(ControlMode.PercentOutput, leftSpeed);
    m_rightMotor.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void setBrakeMode() {
    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotor2.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor2.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
    m_leftMotor.setNeutralMode(NeutralMode.Coast);
    m_leftMotor2.setNeutralMode(NeutralMode.Coast);
    m_rightMotor.setNeutralMode(NeutralMode.Coast);
    m_rightMotor2.setNeutralMode(NeutralMode.Coast);
  }

  public Command arcadeDriveCmd(Supplier<Double> fwdSup, Supplier<Double> rotSup) {
    return run(() -> arcadeDrive(fwdSup.get(), rotSup.get()));
  }

  public Command tankDriveCmd(Supplier<Double> leftSup, Supplier<Double> rightSup) {
    return run(() -> tankDrive(leftSup.get(), rightSup.get()));
  }
}
