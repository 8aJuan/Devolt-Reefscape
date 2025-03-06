package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Commands.singlemotion.releaseCmd;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  // insertar todos los subsistemas necesarios
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Base m_base = new Base();
  private final Climber m_climber = new Climber();

  // The driver's controller
  XboxController m_driverController = new XboxController(0);
  Joystick m_joystick = new Joystick(1);

    private final SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("lv2", m_base.scoreLv2(m_base));
    NamedCommands.registerCommand("lv3", m_base.scoreLv3(m_base));
    NamedCommands.registerCommand("lv4", m_base.scoreLv4(m_base));
    NamedCommands.registerCommand("idle", m_base.idlePositionCmd(m_base));
    NamedCommands.registerCommand("human", m_base.humanPositionCmd(m_base));
    NamedCommands.registerCommand("grab", new RunCommand(() -> m_base.grab(), m_base));
    NamedCommands.registerCommand("release", new ParallelDeadlineGroup(new WaitCommand(.7), new releaseCmd(m_base)));
    NamedCommands.registerCommand("intakeoff", new RunCommand(() -> m_base.intakeOff(), m_base));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);


    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand( // comando default del chasis
        new RunCommand(() -> {
          double turbo = 1;
          if (m_driverController.getLeftTriggerAxis() > 0.3 || m_driverController.getRightTriggerAxis() > 0.3) {
            turbo = 0.2;
          }
          m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY() * turbo, 0.05),
              -MathUtil.applyDeadband(m_driverController.getLeftX() * turbo, 0.05),
              -MathUtil.applyDeadband(m_driverController.getRightX() * turbo, 0.05),
              true);
        }, m_robotDrive));
  }
  // mapeo de botones
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 9).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    new JoystickButton(m_driverController, 1).whileTrue(new RunCommand(() -> m_robotDrive.autoalign(), m_robotDrive));
    new JoystickButton(m_driverController, 2).onTrue(new InstantCommand(()->{
      m_robotDrive.resetPose(new Pose2d(0,0, new Rotation2d(0)) );
      m_robotDrive.followPath(m_robotDrive.getLimelightPath()).schedule();}));

    new JoystickButton(m_driverController, 4).whileTrue(new RunCommand(() -> m_climber.setMotor(1), m_climber));
    new JoystickButton(m_driverController, 3).whileTrue(new RunCommand(() -> m_climber.setMotor(-1), m_climber));


    // intake
    new JoystickButton(m_joystick, 1).whileTrue(new RunCommand(() -> m_base.grab(), m_base));
    new JoystickButton(m_joystick, 2).whileTrue(new RunCommand(() -> m_base.release(), m_base));

    new JoystickButton(m_joystick, 3).onTrue(new InstantCommand(() -> {
      m_base.idlePositionCmd(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 4).onTrue(new InstantCommand(() -> {
      m_base.humanPositionCmd(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 5).onTrue(new InstantCommand(() -> {
      m_base.grabPositionCmd(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 8).onTrue(new InstantCommand(() -> {
      m_base.scoreLv4(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 10).onTrue(new InstantCommand(() -> {
      m_base.scoreLv3(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 11).onTrue(new InstantCommand(() -> {
      m_base.scoreLv1(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 12).onTrue(new InstantCommand(() -> {
      m_base.scoreLv2(m_base).schedule();
    }));
    new JoystickButton(m_joystick, 7).whileTrue(new RunCommand(() -> m_base.algaeLv1(m_base).schedule()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
