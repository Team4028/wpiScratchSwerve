// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.MK4IModuleConstants;

public class SwerveModuleCAN {
  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  //private final RelativeEncoder m_driveEncoder;
  private final WPI_CANCoder m_turningEncoder;
  //private final AnalogInput m_offsetEncoder;

  private double turningMotorOffset;

  private int resetIterations = 0;

  private final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private final int CAN_TIMEOUT_MS = 250;
  private final PIDController m_turningPIDController = new PIDController(0.5, 0, 0.0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModuleCAN(
      int driveMotorChannel,
      int turningMotorChannel,
      int CANEncoderPort,
      double turningMotorOffset) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    m_turningEncoder = new WPI_CANCoder(CANEncoderPort);
    m_driveMotor.configFactoryDefault();
        
    m_driveMotor.setInverted(false);
    m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kDefaultTimeout);
    // m_driveMotor.config_kF(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);
    // m_driveMotor.config_kP(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);
    // m_driveMotor.config_kI(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);
    // m_driveMotor.config_kD(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);  
    // m_driveMotor.config_IntegralZone(0, 0);

    //Setup the Steering Sensor
    m_turningEncoder.configSensorDirection(false);
    m_turningEncoder.configMagnetOffset(Math.toDegrees(turningMotorOffset));
    m_turningEncoder.setPositionToAbsolute();
    //Setup the the closed-loop PID for the steering module loop
    
    m_turningMotor.configFactoryDefault();
    m_turningMotor.configFeedbackNotContinuous(false, Constants.kDefaultTimeout);
    m_turningMotor.configSelectedFeedbackCoefficient(1/Constants.TICKSperTALONFX_DEGREE,0,Constants.kDefaultTimeout);
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,Constants.kDefaultTimeout);

    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,1,Constants.kDefaultTimeout);
    m_turningMotor.configRemoteFeedbackFilter(m_turningEncoder, 0);
    m_turningMotor.configSelectedFeedbackCoefficient(Constants.STEERING_SENSOR_DEGREESperTICKS, 1, Constants.kDefaultTimeout);
    m_turningMotor.configAllowableClosedloopError(Constants.kDefaultPIDSlotID, Constants.kDefaultClosedLoopError, Constants.kDefaultTimeout);
    // m_turningMotor.config_kF(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);
    // m_turningMotor.config_kP(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);
    // m_turningMotor.config_kI(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);
    // m_turningMotor.config_kD(Constants.kDefaultPIDSlotID, 0, Constants.kDefaultTimeout);  
    m_turningMotor.setStatusFramePeriod(
              StatusFrameEnhanced.Status_1_General,
              STATUS_FRAME_GENERAL_PERIOD_MS,
              CAN_TIMEOUT_MS
      );

    //this.m_driveEncoder = new Encoder(driveEncoderPorts[0], driveEncoderPorts[1]);

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setInverted(true);
    

    configMotorPID(m_turningMotor, 0, .2, 0.0, 0.1);
  

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private double getTurningEncoderRadians(){
    double angle = Math.toRadians(m_turningEncoder.getAbsolutePosition()) + turningMotorOffset;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }
    return angle;
    }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(getTurningEncoderRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    checkCanCoderMotorCoder();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        state.speedMetersPerSecond;
    // Calculate the turning motor output from the turning PID controller
    // Calculate the turning motor output from the turning PID controller.
    //m_turningMotor.setSelectedSensorPosition((150/7) * 2048 * getTurningEncoderRadians() / (2 * Math.PI));
    double desiredPulses = state.angle.getDegrees() / 360 * (150/7) * 2048;
    double deltaPulses = desiredPulses - m_turningMotor.getSelectedSensorPosition();


    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    //m_turningMotor.set(ControlMode.Position, getTurnPulses(state.angle.getRadians()));
    setSteeringAngle(state.angle.getDegrees());
  }

  public void configMotorPID(WPI_TalonFX talon, int slotIdx, double p, double i, double d){
    talon.config_kP(slotIdx, p);
    talon.config_kI(slotIdx, i);
    talon.config_kD(slotIdx, d);
    //talon.config_kF(slotIdx, 0.4 * 1023/8360);
    talon.configMotionAcceleration(MK4IModuleConstants.kModuleMaxAccelerationTurningPulsesPer100MsSquared);
    talon.configMotionCruiseVelocity(MK4IModuleConstants.kModuleMaxSpeedTurningPulsesPer100Ms);
  }

//Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }

public double mod(double a, double b){
  var r = a % b;
  if (r < 0) {
      r += b;
  }
  return r;
}
public double minChange(double a, double b, double wrap){
  return halfMod(a - b, wrap);
}

/**
* @return a value in range `[-wrap / 2, wrap / 2)` where `mod(a, wrap) == mod(value, wrap)`
*/
public double halfMod(double a, double wrap) {
  double aa = mod(a, wrap);
  double halfWrap = wrap / 2.0;
  if(aa >= halfWrap){
      aa -= wrap;
  }
  return aa;
}

private void checkCanCoderMotorCoder(){
 if (resetIterations < 1){
   resetIterations ++;
   System.out.println("bruh");
   m_turningMotor.setSelectedSensorPosition(getTurningEncoderRadians() / 2 / Math.PI * (150/7) * 2048);
 }
 }
private double getTurnPulses(double referenceAngleRadians){
  double currentAngleRadians = 2 * Math.PI * m_turningMotor.getSelectedSensorPosition() / MK4IModuleConstants.i_kEncoderCountsPerModuleRev;
  Rotation2d refrot = new Rotation2d(referenceAngleRadians);
  Rotation2d currot = new Rotation2d(currentAngleRadians);
  Rotation2d changerot = refrot.minus(currot);
  return (changerot.getRadians()) / (2.0 * Math.PI) * MK4IModuleConstants.i_kEncoderCountsPerModuleRev + m_turningMotor.getSelectedSensorPosition();
}
public void setSteeringAngle(double _angle){
  //double newAngleDemand = _angle;
  double currentSensorPosition = m_turningMotor.getSelectedSensorPosition();
  double remainder = Math.IEEEremainder(currentSensorPosition, 360);
  double newAngleDemand = _angle + currentSensorPosition -remainder;
 
  //System.out.println(m_turningMotor.getSelectedSensorPosition()-remainder );
  if(newAngleDemand - currentSensorPosition > 180.1){
        newAngleDemand -= 360;
    } else if (newAngleDemand - currentSensorPosition < -180.1){
        newAngleDemand += 360;
    }
    
  m_turningMotor.set(ControlMode.Position, newAngleDemand );
}

public double getSteeringAngle(){
  return m_turningEncoder.getAbsolutePosition();
}

public static SwerveModuleState optimize(
  SwerveModuleState desiredState, Rotation2d currentAngle) {
var delta = desiredState.angle.minus(currentAngle);
if (Math.abs(delta.getDegrees()) > 90.0) {
  return new SwerveModuleState(
      -desiredState.speedMetersPerSecond,
      desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
} else {
  return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
}
}

}
