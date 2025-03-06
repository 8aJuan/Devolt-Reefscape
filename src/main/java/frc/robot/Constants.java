package frc.robot;


import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.8;//velocidad maxima de chasis
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radianes
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Offset angular para cada modulo
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    
    public static final boolean kGyroReversed = true;

    
 
  }

  public static final class CanIds{
        //IDs de intake
        public static final int kIntake1CanId = 1;
        public static final int kIntake2CanId = 2;
        public static final int armCanId = 3;
        public static final int elev1CanId = 4;
        public static final int elev2CanId = 5;
        //IDs de chasis
        public static final int kFrontLeftTurningCanId = 10;
        public static final int kFrontLeftDrivingCanId = 11;
        public static final int kRearLeftTurningCanId = 12;
        public static final int kRearLeftDrivingCanId = 13;
        public static final int kFrontRightTurningCanId = 14;
        public static final int kFrontRightDrivingCanId = 15;
        public static final int kRearRightTurningCanId = 16;
        public static final int kRearRightDrivingCanId = 17;
        //ID para colgarse
        public static final int climberCanid = 6;
  }

  public static final class ModuleConstants {

    public static final int kDrivingMotorPinionTeeth = 14;//dientes del pinion
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;//diametro de ruedas
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class paths {
    public static List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90))
  );

  
  public static PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
  
  public static PathPlannerPath path = new PathPlannerPath(
      waypoints,
      constraints,
      null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
      new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
  );
}
}
