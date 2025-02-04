package frc.robot.Commands.singlemotion;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.DriveSubsystem;

public class alignToAprilTag extends Command {

  private final DriveSubsystem m_drive;

  public alignToAprilTag(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }


   
  @Override
  public void initialize() {
   m_drive.resetPose(null);
  }

  @Override
  public void execute() {
    m_drive.followPath(Constants.paths.path);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}