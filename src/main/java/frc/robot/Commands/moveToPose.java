package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.singlemotion.moveArm;
import frc.robot.Commands.singlemotion.moveElevator;
import frc.robot.Commands.singlemotion.moveWrist;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class moveToPose extends ParallelCommandGroup {

  
  public moveToPose(Arm m_arm, Wrist m_wrist, Elevator m_elevator, double[] pose) { 
    addCommands(
      new moveArm(m_arm, pose[0]), 
      new moveWrist(m_wrist, pose[1]), 
      new moveElevator(m_elevator, pose[2])
      );
  }
}
