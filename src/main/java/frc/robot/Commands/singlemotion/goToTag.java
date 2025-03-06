
package frc.robot.Commands.singlemotion;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class goToTag extends SequentialCommandGroup {
  PathPlannerPath path;
  public goToTag(DriveSubsystem m_drive) {
    addCommands(
      new InstantCommand(()->{m_drive.resetPose(null);}),
      new InstantCommand(()->{path = m_drive.getLimelightPath();}),
      new WaitCommand(.02),
      new InstantCommand(()->{m_drive.followPath(path).schedule();})
    );
  }
}
