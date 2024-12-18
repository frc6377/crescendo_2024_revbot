package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.utilities.DebugEntry;

public class DriveTrainSubSystem extends SubsystemBase {
  private final VictorSPX m_leftMotor = new VictorSPX(DriveConstants.LEFT_MOTOR_PORT);
  private final VictorSPX m_leftMotor2 = new VictorSPX(DriveConstants.LEFT_MOTOR_PORT_2);
  private final VictorSPX m_rightMotor = new VictorSPX(DriveConstants.RIGHT_MOTOR_PORT);
  private final VictorSPX m_rightMotor2 = new VictorSPX(DriveConstants.RIGHT_MOTOR_PORT_2);
  private DebugEntry<Double> m_driveMotor1CurrentVelocity;
  private DebugEntry<Double> m_driveMotor2CurrentVelocity;
  private DebugEntry<Double> m_driveMotor3CurrentVelocity;
  private DebugEntry<Double> m_driveMotor4CurrentVelocity;
  private DifferentialDrive m_robotDrive;

  public DriveTrainSubSystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    m_rightMotor2.setInverted(true);

    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

    SendableRegistry.addLW(this, "DriveTrainSubSystem");

    m_robotDrive =
        new DifferentialDrive(
            speed -> m_leftMotor.set(ControlMode.PercentOutput, speed),
            speed -> m_rightMotor.set(ControlMode.PercentOutput, speed));

    m_driveMotor1CurrentVelocity = new DebugEntry<Double>(0.0, "driveMotor1", this);
    m_driveMotor2CurrentVelocity = new DebugEntry<Double>(0.0, "driveMotor2", this);
    m_driveMotor3CurrentVelocity = new DebugEntry<Double>(0.0, "driveMotor3", this);
    m_driveMotor4CurrentVelocity = new DebugEntry<Double>(0.0, "driveMotor4", this);
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

  @Override
  public void periodic() {
    m_driveMotor1CurrentVelocity.log(m_leftMotor.getMotorOutputVoltage());
    m_driveMotor2CurrentVelocity.log(m_leftMotor2.getMotorOutputVoltage());
    m_driveMotor3CurrentVelocity.log(m_rightMotor.getMotorOutputVoltage());
    m_driveMotor4CurrentVelocity.log(m_rightMotor2.getMotorOutputVoltage());
  }
}
