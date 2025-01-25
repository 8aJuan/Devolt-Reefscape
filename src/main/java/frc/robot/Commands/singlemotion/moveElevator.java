package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class moveElevator extends Command {

  private final Elevator m_elevator;
  private final PIDController pid;

  public moveElevator(Elevator m_elevator, double setpoint) {
    this.m_elevator = m_elevator;
    this.pid = new PIDController(.1, 0, 0);
    pid.setSetpoint(setpoint);
    addRequirements(m_elevator);
  }
  
  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    double output = pid.calculate(m_elevator.getEncoder());
    m_elevator.setMotors(output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
