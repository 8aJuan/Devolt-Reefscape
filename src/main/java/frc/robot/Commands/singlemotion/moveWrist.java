
package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class moveWrist extends Command {

  private final Wrist m_wrist;
  private final PIDController pid;
  private double target;

  public moveWrist(Wrist m_wrist, double setpoint) {
    this.m_wrist = m_wrist;
    this.pid = new PIDController(.05, 0, 0);
    pid.setSetpoint(setpoint);
    addRequirements(m_wrist);
    target = setpoint;
  }
  
  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    double output = pid.calculate(m_wrist.getEncoder());
    if (output > .5){
      m_wrist.setMotor(.5);
    }
    else if (output<-.5){
      m_wrist.setMotor(-.5);
    }
    else{
      m_wrist.setMotor(output);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Math.abs(m_wrist.getEncoder()-target)<.5){
      return true;
    }
    return false;
  }
}
