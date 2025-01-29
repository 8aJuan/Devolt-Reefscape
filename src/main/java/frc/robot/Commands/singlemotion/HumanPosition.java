
package frc.robot.Commands.singlemotion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Base;

public class HumanPosition extends SequentialCommandGroup {

  public HumanPosition(Base m_base) {
    addCommands(
      new moveArm(m_base.m_arm, 15),
      new moveWrist(m_base.m_wrist, 115),
      new moveArm(m_base.m_arm, 0)
    );
  }
}
