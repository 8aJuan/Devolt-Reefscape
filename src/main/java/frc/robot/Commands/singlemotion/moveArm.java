package frc.robot.Commands.singlemotion;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


public class moveArm extends Command {

  private final Arm m_arm;
  private final PIDController pid;
  private double target;

  public moveArm(Arm m_arm, double setpoint) {
    this. m_arm = m_arm;
    this.pid = new PIDController(.1, 0, 0);
    pid.setSetpoint(setpoint);
    addRequirements(m_arm);
    target = setpoint;
  }

  @Override
  public void initialize() {

    pid.reset();
  }

  @Override
  public void execute() {
    double output = pid.calculate(m_arm.getEncoder());
    m_arm.setMotor(output);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(Math.abs(m_arm.getEncoder()-target)<.3){
      return true;
    }
    return false;
  }
}
