// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class DriveConstants {
    public static final int LEFT_MOTOR_PORT = 1;
    public static final int LEFT_MOTOR_PORT_2 = 2;
    public static final int RIGHT_MOTOR_PORT = 3;
    public static final int RIGHT_MOTOR_PORT_2 = 4;
    public static final int INTAKE_MOTOR_PORT = 5;
    public static final int INTAKE_MOTOR_PORT_2 = 6;
    public static final int SHOOTER_MOTOR_PORT = 7;
    public static final int SHOOTER_MOTOR_PORT_2 = 8;
  }

  public static class ShooterConstants {
    public static final int TIME_TO_SHOOT = 2;
    public static final double SHOOTER_P = 0.1;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_FF = 0;
    public static final double SHOOTER_TOLERANCE = 0.1;
    public static final double REVERSE_PERCENT = -0.1;
    public static final double SHOOTER_RPM = 50;
  }

  public static class IntakeConstants {
    public static final double FIRST_MOTOR_VELOCITY = 20;
    public static final double SECOND_MOTOR_VELOCITY = 40;
    public static final double INTAKE_TOLERANCE = 0.1;
    public static final double INTAKE_P = 0.1;
    public static final double INTAKE_I = 0;
    public static final double INTAKE_D = 0;
    public static final double INTAKE_FF = 0;
  }
}
