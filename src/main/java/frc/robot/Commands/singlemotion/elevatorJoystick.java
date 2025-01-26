
package frc.robot.Commands.singlemotion;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class elevatorJoystick extends Command {

  private final Elevator m_elevator;
  private Supplier<Double> output;

  public elevatorJoystick(Elevator m_elevator, Supplier<Double> output) {
   this.m_elevator = m_elevator;
   this.output = output;
   addRequirements(m_elevator);
   

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realtimeoutput = output.get()/2;
    m_elevator.setMotors(realtimeoutput);
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
