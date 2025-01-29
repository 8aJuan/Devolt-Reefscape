package frc.robot.Commands.singlemotion;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class moveArm extends Command {

  private final Arm m_arm;
  private double target;

  public moveArm(Arm m_arm, double setpoint) {
    this.m_arm = m_arm; //igualar subsistema al introducido al comando
    addRequirements(m_arm); 
    target = setpoint;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() { //no superar velocidad de .5
    m_arm.moveTo(target);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setMotor(0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getControllerError()) < 1;
  }

}
