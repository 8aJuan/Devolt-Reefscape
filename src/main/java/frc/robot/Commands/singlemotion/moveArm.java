package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class moveArm extends Command {

  private final Arm m_arm;
  private final PIDController pid;
  private double target;

  public moveArm(Arm m_arm, double setpoint) {
    this. m_arm = m_arm; //igualar subsistema al introducido al comando
    this.pid = new PIDController(.02, 0, 0);
    pid.setSetpoint(setpoint); //objetivo
    addRequirements(m_arm); 
    target = setpoint;
  }

  @Override
  public void initialize() {
    pid.reset(); //eliminar remanentes de ki
  }

  @Override
  public void execute() { //no superar velocidad de .5
    double output = pid.calculate(m_arm.getEncoder());
    if (output > .5){
      m_arm.setMotor(.5);
    }else if(output < -.5){
      m_arm.setMotor(-.5);
    }else{
      m_arm.setMotor(output);
    }
   
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(Math.abs(m_arm.getEncoder()-target)<.5){
      return true;
    }
    return false;
  }
}
