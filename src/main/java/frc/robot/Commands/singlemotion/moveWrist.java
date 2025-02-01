
package frc.robot.Commands.singlemotion;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class moveWrist extends Command {

  private final Wrist m_wrist;
  private double target;

  public moveWrist(Wrist m_wrist, double setpoint) {
    this.m_wrist = m_wrist;

    addRequirements(m_wrist);
    target = setpoint;
  }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_wrist.moveTo(target);
  }

  @Override
  public void end(boolean interrupted) {
    m_wrist.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_wrist.getControllerError()) < 3;
  }
}
