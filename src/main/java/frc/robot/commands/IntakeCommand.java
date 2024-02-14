package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {

  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  public IntakeCommand(IntakeSubsystem IntakeSubsystem, ShooterSubsystem ShooterSubsystem) {
    m_ShooterSubsystem = ShooterSubsystem;
    m_IntakeSubsystem = IntakeSubsystem;
    addRequirements(ShooterSubsystem, IntakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_IntakeSubsystem.runIntake();
    m_ShooterSubsystem.reverseShooter();
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopMotors();
    m_ShooterSubsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
