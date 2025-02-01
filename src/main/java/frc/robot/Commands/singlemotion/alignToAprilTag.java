package frc.robot.Commands.singlemotion;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class alignToAprilTag extends Command {

  private final DriveSubsystem m_drive;
  private double[] visionPose;
  List<Waypoint> waypoints;
  PathPlannerPath path;


  public alignToAprilTag(DriveSubsystem m_drive) {
    this.m_drive = m_drive;


    addRequirements(m_drive);
  }

   
  @Override
  public void initialize() {
    visionPose = m_drive.getLimelightPose();
    waypoints = PathPlannerPath.waypointsFromPoses(
        m_drive.getPose(),
        new Pose2d(m_drive.getPose().getTranslation().plus(new Translation2d(1,1)), Rotation2d.fromDegrees(0))
    );
    path = new PathPlannerPath(
      waypoints,null,null,
      new GoalEndState(0.0, Rotation2d.fromDegrees(0)));
      path.preventFlipping = true;
  }

  @Override
  public void execute() {
    try{
    AutoBuilder.followPath(path);
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
