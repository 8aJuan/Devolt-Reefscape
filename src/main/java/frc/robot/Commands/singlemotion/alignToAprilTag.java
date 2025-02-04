package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class alignToAprilTag extends Command {

  private final DriveSubsystem m_drive;
 
  public alignToAprilTag(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }


   
  @Override
  public void initialize() {
    m_drive.resetPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0)));
  }

  @Override
  public void execute() {
    try{
    
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        Commands.none();
    }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
