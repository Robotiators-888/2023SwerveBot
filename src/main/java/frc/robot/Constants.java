package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.libs.PIDGains;

public final class Constants {

  public static class Swerve {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians

    public static final double kTurningVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
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
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class Drivetrain {
    public static final int kFRONT_LEFT_DRIVE_MOTOR_CANID = 20;
    public static final int kFRONT_LEFT_STEER_MOTOR_CANID = 21;
    public static final int kFRONT_RIGHT_DRIVE_MOTOR_CANID = 22;
    public static final int kFRONT_RIGHT_STEER_MOTOR_CANID = 23;
    public static final int kBACK_LEFT_DRIVE_MOTOR_CANID = 24;
    public static final int kBACK_LEFT_STEER_MOTOR_CANID = 25;
    public static final int kBACK_RIGHT_DRIVE_MOTOR_CANID = 26;
    public static final int kBACK_RIGHT_STEER_MOTOR_CANID = 27;

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 100000; // radians per second
    public static final double kMagnitudeSlewRate = 100000; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = true;
    }
 
    public static class Manuiplator{
        public static final int kMANUIP_ROTATE_MOTOR_CANID = 31;
        public static final int kMANUIP_INTAKE_MOTOR_CANID = 32;
        public static final int kMANUIP_EXTEND_MOTOR_CANID = 33;

        public static final double INTAKE_CONE_SPEED = 0.65;
        public static final double INTAKE_CUBE_SPEED = 0.55;

        public static final double PID_kP = 0.11425;
        public static final int PID_kI = 0;
        public static final double PID_kD = 0.021;
        public static final double FF_kA = .1267;
        public static final double FF_kG = .4847; //amount of volts to Overcome gravity on the arm, was 1
        public static final double FF_kS = .11092;
        public static final double FF_kV = 5.9339; 
        public static final double FF_Velocity = 0.90511;
        public static final double FF_Accel = 0.68018;
        public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(0.75, 1);
        public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(FF_kS, FF_kG, FF_kV, FF_kA);

        public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);

        //public static final double kGroundPosition = 0.1;
        public static final double kGroundPosition = .07;
        public static final double kScoreHigh = .6;
        public static final double kScoreMid = .335;
        public static final double kScoreLow = 0.1;
        public static final double kStow = .99;
        public static final double kSingleFeeder = 0.38;
        

        public static final double kmaxVelocity = 6.47*Math.PI;
        public static final double kmaxAcceleration = 4.27;
        
    }

    public static class Extension{
        public static final double kForwardSpeed = 0.5; 
        public static final double kReverseSpeed = -0.5; 

    }

    }


