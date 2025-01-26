package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.singlemotion.elevatorJoystick;
import frc.robot.Commands.singlemotion.moveArm;
import frc.robot.Commands.singlemotion.moveElevator;
import frc.robot.Commands.singlemotion.moveWrist;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
//import frc.robot.subsystems.Base;
import frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

public class RobotContainer {
  // insertar todos los subsistemas necesarios
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final Base m_base = new Base();
  private final Arm m_arm = new Arm();
  private final Elevator m_elevator = new Elevator();
  private final Wrist m_wrist = new Wrist();
  
 

  // The driver's controller
  XboxController m_driverController = new XboxController(0);
  Joystick m_joystick = new Joystick(1);

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // comando default del chasis
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

    //m_base.setDefaultCommand(new RunCommand(() -> {m_base.moveTo();}, m_base)); //mantener posicion de mecanismos
      m_elevator.setDefaultCommand(new elevatorJoystick(m_elevator, ()-> -m_joystick.getRawAxis(1)));
        }
         


   //mapeo de botones
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 9).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(),m_robotDrive));
    /*new JoystickButton(m_driverController, 1).whileTrue(new moveArm(m_arm, 80));

    new JoystickButton(m_joystick, 5).whileTrue(new RunCommand(()-> m_base.grabPosition(), m_base));
    new JoystickButton(m_joystick, 3).whileTrue(new RunCommand(()-> m_base.idlePosition(), m_base));
    new JoystickButton(m_joystick, 6).whileTrue(new RunCommand(()-> m_base.toggleGamePiece(), m_base));
    new JoystickButton(m_joystick, 4).whileTrue(new RunCommand(()-> m_base.humanPosition(), m_base));
    */
    new JoystickButton(m_joystick, 7).whileTrue(new moveArm(m_arm, 0));
    new JoystickButton(m_joystick, 8).whileTrue(new moveArm(m_arm, 10));
    new JoystickButton(m_joystick, 9).whileTrue(new moveWrist(m_wrist, 0));
    new JoystickButton(m_joystick, 10).whileTrue(new moveWrist(m_wrist, 100));
    new JoystickButton(m_joystick, 11).whileTrue(new moveElevator(m_elevator, 0));
    new JoystickButton(m_joystick, 12).whileTrue(new moveElevator(m_elevator, 15));




    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
