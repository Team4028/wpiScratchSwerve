// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final boolean MK4I = true;
    public static final boolean isNAVX = true;

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int i_kFrontLeftDriveMotorPort = 2;
    public static final int i_kRearLeftDriveMotorPort = 6;
    public static final int i_kFrontRightDriveMotorPort = 4;
    public static final int i_kRearRightDriveMotorPort = 8;

    public static final int i_kFrontLeftTurningMotorPort = 1;
    public static final int i_kRearLeftTurningMotorPort = 5;
    public static final int i_kFrontRightTurningMotorPort = 3;
    public static final int i_kRearRightTurningMotorPort = 7;

    public static final int i_kFrontLeftEncoderCan = 1;
    public static final int i_kRearLeftEncoderCan = 3;
    public static final int i_kFrontRightEncoderCan = 2;
    public static final int i_kRearRightEncoderCan = 4;

    public static final int pigeonCan = 1;


    public static final double kTrackWidth = util.inchesToMeters(23.75 - 2.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = util.inchesToMeters(25.75 - 2.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false; //true for mk2 chassis

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;

    public static final double kMaxSpeedMetersPerSecond = util.feetToMeters(12.0);
    public static final double i_kMaxSpeedMetersPerSecond = util.feetToMeters(16.3);
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 42;
    public static final double kWheelDiameterMeters = util.inchesToMeters(4.0);
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) * (1.0 / (60.0 / 15.0) / (20.0 / 24.0) / (40.0 / 16.0));

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 0.5;

    public static final double kPModuleDriveController = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class MK4IModuleConstants {
    public static final double i_kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double i_kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int i_kEncoderCPR = 4096;
    public static final double i_kWheelDiameterMeters = util.inchesToMeters(4.0);
    public static final double i_kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (i_kWheelDiameterMeters * Math.PI) * (1.0 / (50.0 / 14.0) / (17.0 / 27.0) / (45.0 / 15.0));

    public static final double i_kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) i_kEncoderCPR;

    public static final double i_kPModuleTurningController = 0.2;

    public static final double i_kPModuleDriveController = 0;

    public static final double i_kEncoderCountsPerModuleRev = (150/7) * 2048;

    public static final double kModuleMaxSpeedTurningRadiansPerSecond = 16*Math.PI;
    public static final double kModuleMaxAccelerationTurningRadiansPerSecondSquared = 256*Math.PI;
    public static final double kModuleMaxSpeedTurningPulsesPer100Ms = kModuleMaxSpeedTurningRadiansPerSecond * i_kEncoderCountsPerModuleRev * 0.1;
    public static final double kModuleMaxAccelerationTurningPulsesPer100MsSquared = kModuleMaxAccelerationTurningRadiansPerSecondSquared * i_kEncoderCountsPerModuleRev * 0.01;
  }


  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = util.feetToMeters(6);
    public static final double kMaxAccelerationMetersPerSecondSquared = util.feetToMeters(4);
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0;
    public static final double kPYController = 0;
    public static final double kPThetaController = 0;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  //TEST CONSTANTS STUF
  public static final double WHEEL_RADIUS_METERS = util.inchesToMeters(4.0);
  public static final double WHEEL_BASE_METERS = 22.5 * 2.54/100; //18 inch wheel base to meters track width is 24in and wheel base is 22.5 in
  public static final double MAX_SPEED_TICKSper100MS = 21900;
  public static final double STEERING_MOTOR_GEARING = 150/7;
  public static final double DRIVE_MOTOR_GEARING = 6.75;
  public static final double SPEED_GOVERNOR =.51; //.11 is a good safe start. Unlock it to "1" when you're confident with the robot
  public static final double TRACK_WIDE = 24 * 2.54/100;
  
  
  //SWERVE Drive Default Values
  public static final double ROBOTHoldAngleKP = 15; //Start at .7 and see where you go from there
  public static final boolean DEFAULT_HOLD_ROBOT_ANGLE = false;
public static final boolean DEFAULT_FIELD_RELATIVE_DRIVE = false;
public static final double DEFAULT_HOLD_ROBOT_ANGLE_SETPOINT = 0; 

  //Swerve Drive Motor IDs
  public static final int FRDriveID = 8;
  public static final int FLDriveID = 15;
  public static final int BRDriveID = 10;
  public static final int BLDriveID = 6;

  //Swerve Steer Motor IDs
  public static final int FRSteerID = 7;
  public static final int FLSteerID = 5;
  public static final int BRSteerID = 9;
  public static final int BLSteerID = 11;

  //Swerve CANCoder Sensor IDs
  public static final int FRSensorID = 2;
  public static final int FLSensorID = 4;
  public static final int BRSensorID = 1;
  public static final int BLSensorID = 3;

  //Swerve CANCoder Sensort offsets
  //CHANGE TO 0 first, reset the sensor, 
  //PHYSICALLY zero out the motor 
  //place the OPPOSITE of the value
  public static double FRSensorOffset = -72.598;
  public static double FLSensorOffset = -36.035;
  public static double BRSensorOffset = -66.533;
  public static double BLSensorOffset = 37.529;


  //Give a positive input on the joystick or phoenix tuner
  //Switch this if it goes opposite the desired direction
  //Because of the gearing the convention could be reversed (GUESS AND CHECK)
  public static TalonFXInvertType FRInvertType = TalonFXInvertType.Clockwise;
  public static TalonFXInvertType FLInvertType = TalonFXInvertType.CounterClockwise;
  public static TalonFXInvertType BRInvertType = TalonFXInvertType.Clockwise;
  public static TalonFXInvertType BLInvertType = TalonFXInvertType.CounterClockwise;

  //Swerve Steering PIDs (kP, kI, kD)
  public static Gains FRSteerGains = new Gains(25, 0, 0);
  public static Gains FLSteerGains = new Gains(25, 0, 0);
  public static Gains BRSteerGains = new Gains(25, 0, 0);
  public static Gains BLSteerGains = new Gains(25, 0, 0);

  //Swerve Driving PIDs (kP, kI, kD)
  //Once characterized the drive PIDs are meaningless
  public static Gains FRDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
  public static Gains FLDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
  public static Gains BRDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
  public static Gains BLDriveGains = new Gains(0.07, 0, 0, 1023.0/20660.0);
  public static final double kS = 0.0;//0.4438;//0.60968;
  public static final double kV = 0.0;//2.337;//2.2691;
  public static final double kA = 0.0;//0.29114;//0.19335;
  public static final double kP = 0.0;//3.325;

  //CTRE CAN-based constants (shouldn't need to change these)
  public static final int kDefaultPIDSlotID = 0;
  public static final int kDefaultTimeout = 30;//milliseconds
  public static final int kDefaultClosedLoopError = 1; //degrees 
  
  //Constants for conversion maths (RARELY THESE SHOULD BE CHANGED)
  public static final double SECONDSper100MS = .1;
  public static final double STEERING_SENSOR_TICKSperROTATION = 4096;
  public static final double STEERING_SENSOR_DEGREESperTICKS = 360/STEERING_SENSOR_TICKSperROTATION;
  public static final double TICKSperTALONFX_Rotation = 2048;
  public static final double DRIVE_MOTOR_TICKSperREVOLUTION = DRIVE_MOTOR_GEARING*TICKSperTALONFX_Rotation;
  public static final double METERSperWHEEL_REVOLUTION = 2*Math.PI*WHEEL_RADIUS_METERS;
  public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*pythagoreanTheorem(TRACK_WIDE, WHEEL_BASE_METERS);
  public static final double MAX_SPEED_METERSperSECOND = MAX_SPEED_TICKSper100MS/SECONDSper100MS/DRIVE_MOTOR_TICKSperREVOLUTION*METERSperWHEEL_REVOLUTION;
  public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND/METERSperROBOT_REVOLUTION*(2*Math.PI);
  public static final double TICKSperTALONFX_DEGREE = TICKSperTALONFX_Rotation*STEERING_MOTOR_GEARING/360;

  public static class Gains {
      public final double kP;
      public final double kI;
      public final double kD;
      public final double kF;
      public final int kIzone;
      public final double kPeakOutput;
      /**
       * @param _kP
       * @param _kI
       * @param _kD
       */
      public Gains(double _kP, double _kI, double _kD){
          kP = _kP;
          kI = _kI;
          kD = _kD;
          kF = 0;
          kIzone = 0;
          kPeakOutput = 1;
      }
      public Gains(double _kP, double _kI, double _kD, double _kF){
          kP = _kP;
          kI = _kI;
          kD = _kD;
          kF = _kF;
          kIzone = 300;
          kPeakOutput = 1;
      }
  }

  private static double pythagoreanTheorem(double side1, double side2) {
      double radius = Math.sqrt(Math.pow(side1, 2) + Math.pow(side2, 2));
      return radius;
  }
}

