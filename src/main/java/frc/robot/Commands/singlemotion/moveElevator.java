package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class moveElevator extends Command {

  private final Elevator m_elevator;
  private final PIDController upPid;
  private final PIDController downPid;
  private final double target;

  public moveElevator(Elevator m_elevator, double setpoint) {
    this.m_elevator = m_elevator;
    this.target = setpoint;
    downPid = new PIDController(.001, 0, 0);
    upPid = new PIDController(.01, 0, 0);
    downPid.setSetpoint(setpoint);
    upPid.setSetpoint(setpoint);
    addRequirements(m_elevator);
  }
  
  @Override
  public void initialize() {
    upPid.reset();
    downPid.reset();
  }

  @Override
  public void execute() {
    double output;
    if(target > m_elevator.getEncoder()){
      output = upPid.calculate(m_elevator.getEncoder());
    }else{
      output = downPid.calculate(m_elevator.getEncoder());
    }
    m_elevator.setMotors(output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (Math.abs(m_elevator.getEncoder()-target)<.5){
      return true;
    }
    
    return false;
  }
}
