
package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class moveWrist extends Command {

  private final Wrist m_wrist;
  private final PIDController pid;

  public moveWrist(Wrist m_wrist, double setpoint) {
    this.m_wrist = m_wrist;
    this.pid = new PIDController(.1, 0, 0);
    pid.setSetpoint(setpoint);
    addRequirements(m_wrist);
  }
  
  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    double output = pid.calculate(m_wrist.getEncoder());
    m_wrist.setMotor(output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
