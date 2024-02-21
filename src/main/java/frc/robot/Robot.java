// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrainSubSystem;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
  public static final boolean isCompetition = false;
  private XboxController m_DriverJoystick;
  private final DriveTrainSubSystem m_driveTrain = new DriveTrainSubSystem();

  @Override
  public void robotInit() {
    m_DriverJoystick = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    m_driveTrain.setBrakeMode();
  }

  public void teleopPeriodic() {
    double forward = -m_DriverJoystick.getLeftY();
    double rotation = m_DriverJoystick.getRightX();
    m_driveTrain.arcadeDrive(forward, rotation);
  }
}
