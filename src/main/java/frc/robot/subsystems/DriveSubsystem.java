package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Modulos de swerve
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CanIds.kFrontLeftDrivingCanId,
      CanIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CanIds.kFrontRightDrivingCanId,
      CanIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CanIds.kRearLeftDrivingCanId,
      CanIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CanIds.kRearRightDrivingCanId,
      CanIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI); // giroscopo

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry( // odometria
      DriveConstants.kDriveKinematics,

      Rotation2d.fromDegrees(-navx.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  public RobotConfig dconfig; // crear configuracion de robot para pathfinder

  public DriveSubsystem() {
    try {
      dconfig = RobotConfig.fromGUISettings(); // extraer configuracion de aplicacion de pathfinder
    } catch (Exception e) {
      e.printStackTrace();
    }

    // configurar autobuilder
    AutoBuilder.configure(
        this::getPose, // supplier de pose2d del robot
        this::resetPose, // metodo para reiniciar la oometria
        this::getRobotRelativeSpeeds, // supplier para velocidades del chasis
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Metodo para mover el chasis
        new PPHolonomicDriveController( // controlador de movimiento
            new PIDConstants(5.0, 0.0, 0.0), // PID de traslacion
            new PIDConstants(5.0, 0.0, 0.0) // PID de rotacion
        ),
        dconfig, // configuracion de robot
        () -> {
          return false;
        },
        this // este subsistema de chasis
    );

  }

  @Override
  // se actualiza la odometria y se introduce angulo de navx a smartdashboard
  public void periodic() {

    m_odometry.update(
        Rotation2d.fromDegrees(-navx.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  // devuelve pose 2d del robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // actualiza pose2d del robot a la introducida
  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-navx.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  // funcion de mover el chasis con las velocidades introducidas
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-navx.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // poner las ruedas en x para evitar movimiento
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // reset del navx
  public void zeroHeading() {
    navx.zeroYaw();
  }

  // metodo demo de alineacion con apriltag usando limelight
  public void autoalign() {
    double[] position = LimelightHelpers.getBotPose_TargetSpace(null);
    double x = -position[2];
    double y = position[0];
    double rot = LimelightHelpers.getTX(null);
    drive((x * .1), (y * .2), (-rot * .02), false);

  }

  public Command allignToTag(DriveSubsystem m_drive) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        m_drive.getPose(), // pose inicial
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

    PathPlannerPath path = new PathPlannerPath(
        waypoints, null,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)));

    path.preventFlipping = true;

    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_drive.resetPose(new Pose2d(new Translation2d(-.5, -.5), new Rotation2d(0)));
        }),
        new InstantCommand(() -> {
          try {
            AutoBuilder.followPath(path);
          } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            Commands.none();
          }
        }));
  }

  public double[] getLimelightPose() {
    return LimelightHelpers.getBotPose_TargetSpace(null);
  }

  // devuelve rotation2d con el angulo actual de navx
  public double getHeading() {
    return Rotation2d.fromDegrees(-navx.getAngle()).getDegrees();
  }

  // devuelve double con velocidad de giro
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  // devuelve estado de los modulos de swerve
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  // se introducen los estados de modulos para devolver velocidades de chasis
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  // funcion para movimiento usando Chassisspeeds como input, uso en pathplanner
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  // movimiento relativo al robot usando chassisspeeds
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  public void stop() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_rearLeft.stop();
    m_rearRight.stop();
  }

  public FollowPathCommand followPath(PathPlannerPath path) {
    return new FollowPathCommand(
        path,
        this::getPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        this.dconfig,
        null,
        this
      );
  }
}