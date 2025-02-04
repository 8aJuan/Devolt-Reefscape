package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Base;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  // insertar todos los subsistemas necesarios
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Base m_base = new Base();

  

  // The driver's controller
  XboxController m_driverController = new XboxController(0);
  Joystick m_joystick = new Joystick(1);

  public RobotContainer() {

    NamedCommands.registerCommand("iddlePositionLeft", m_base.idlePositionLeftCmd(m_base));

    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand( // comando default del chasis
      new RunCommand(() -> {
        double turbo = 1;
        if(m_driverController.getLeftTriggerAxis() > 0.3 || m_driverController.getRightTriggerAxis() > 0.3) {
          turbo = 0.2;
          }
          m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController.getLeftY() * turbo,0.05),
            -MathUtil.applyDeadband(m_driverController.getLeftX() * turbo,0.05),
            -MathUtil.applyDeadband(m_driverController.getRightX() * turbo, 0.05),
            true); 
            },m_robotDrive)); 

    ;
        }
         
   //mapeo de botones
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 9).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));
    new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_robotDrive.autoalign(),m_robotDrive));

      
      
    

    //intake 
    new JoystickButton(m_joystick, 1).whileTrue(new RunCommand(() -> m_base.grab(), m_base));
    new JoystickButton(m_joystick, 2).whileTrue(new RunCommand(() -> m_base.release(), m_base));
    
    new JoystickButton(m_joystick, 3).onTrue(new InstantCommand(()->{m_base.idlePositionCmd(m_base).schedule();}));
    new JoystickButton(m_joystick, 4).onTrue(new InstantCommand(()->{m_base.humanPositionCmd(m_base).schedule();}));
    new JoystickButton(m_joystick, 5).onTrue(new InstantCommand(()->{m_base.grabPositionCmd(m_base).schedule();}));
    new JoystickButton(m_joystick, 6).onTrue(new InstantCommand(()-> m_base.toggleGamePiece(), m_base));
    new JoystickButton(m_joystick, 8).onTrue(new InstantCommand(()->{m_base.scoreLv4(true, m_base).schedule();}));
    new JoystickButton(m_joystick, 10).onTrue(new InstantCommand(()->{m_base.scoreLv3(true, m_base).schedule();}));
    new JoystickButton(m_joystick, 11).onTrue(new InstantCommand(()->{m_base.idlePositionLeftCmd(m_base).schedule();}));
    new JoystickButton(m_joystick, 12).onTrue(new InstantCommand(()->{m_base.scoreLv2(true, m_base).schedule();}));
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Autonomo");
  }
}
