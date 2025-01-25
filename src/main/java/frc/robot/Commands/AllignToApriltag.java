
package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;


public class AllignToApriltag extends Command {

  private final DriveSubsystem m_drive;
  private final PIDController xpid;
  private final PIDController ypid;
  private final PIDController rotpid;
  double[] reefPositiionD = {-.1, 0, 0}; //{offset hacia enfrente, offset de lado, offset angular}

  public AllignToApriltag(DriveSubsystem m_drive) {
    m_drive.resetEncoders();
    double[] position = LimelightHelpers.getBotPose(null);
    this.m_drive = m_drive;

    this.xpid = new PIDController(.1, 0, 0);
    this.ypid = new PIDController(.1, 0, 0);
    this.rotpid = new PIDController(.01, 0, 0);

    xpid.setSetpoint(-position[2]+ reefPositiionD[0]);
    ypid.setSetpoint(-position[0]+ reefPositiionD[1]);
    rotpid.setSetpoint(m_drive.getHeading()+position[4] + reefPositiionD[2]);

    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xpid.reset();
    ypid.reset();
    rotpid.reset();
    m_drive.resetOdometry(null);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOutput = xpid.calculate(m_drive.getPose().getX());
    double yOutput = ypid.calculate(m_drive.getPose().getY());
    double rotOutput = rotpid.calculate(m_drive.getHeading());

    m_drive.drive(xOutput, yOutput, rotOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
