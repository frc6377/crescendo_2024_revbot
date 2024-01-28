// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends LoggedRobot {
  private DifferentialDrive m_robotDrive;
  private XboxController m_DriverJoystick;

  private final VictorSPX m_leftMotor = new VictorSPX(DriveConstants.kLeftMotorPort);
  private final VictorSPX m_leftMotor2 = new VictorSPX(DriveConstants.kLeftMotorPort2);
  private final VictorSPX m_rightMotor = new VictorSPX(DriveConstants.kRightMotorPort);
  private final VictorSPX m_rightMotor2 = new VictorSPX(DriveConstants.kRightMotorPort2);

  @Override
  public void robotInit() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor2);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor2);

    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_leftMotor2.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor2.setNeutralMode(NeutralMode.Brake);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    m_rightMotor2.setInverted(true);

    m_robotDrive =
        new DifferentialDrive(
            speed -> m_leftMotor.set(ControlMode.PercentOutput, speed),
            speed -> m_rightMotor.set(ControlMode.PercentOutput, speed));

    m_DriverJoystick = new XboxController(DriveConstants.driverJoystickPort);
  }

  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_DriverJoystick.getLeftY(), -m_DriverJoystick.getRightY());
  }
}
