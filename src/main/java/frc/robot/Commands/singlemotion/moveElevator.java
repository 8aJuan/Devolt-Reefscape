package frc.robot.Commands.singlemotion;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class moveElevator extends Command {

  private final Elevator m_elevator;

  private final double target;

  public moveElevator(Elevator m_elevator, double setpoint) {
    this.m_elevator = m_elevator;
    this.target = setpoint;
    addRequirements(m_elevator);
  }
  
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_elevator.moveTo(target);
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.setMotors(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_elevator.getControllerError()) < 2;
}
}