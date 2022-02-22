// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.util;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MK4IModuleConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModuleCAN {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private final WPI_CANCoder m_turningEncoder;

  private int resetIterations = 0;

  private final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
  private final int CAN_TIMEOUT_MS = 250;

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
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);
    m_turningEncoder = new WPI_CANCoder(CANEncoderPort);
    m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 0);
    m_turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    m_turningEncoder.configMagnetOffset(Math.toDegrees(turningMotorOffset));
    m_turningMotor.configRemoteFeedbackFilter(m_turningEncoder, 0);
    m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 0);
    m_turningMotor.configSelectedFeedbackCoefficient(1);
    m_turningEncoder.setPositionToAbsolute();
    m_turningMotor.setStatusFramePeriod(
              StatusFrameEnhanced.Status_1_General,
              STATUS_FRAME_GENERAL_PERIOD_MS,
              CAN_TIMEOUT_MS
      );

    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningMotor.setInverted(true);
    

    configMotorPID(m_turningMotor, 0, 1.2, 0.0, 0);
  }

  private double getTurningEncoderRadians(){
    return Math.toRadians(m_turningMotor.getSelectedSensorPosition(0) / 4096 * 360);
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
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(m_turningMotor.getSelectedSensorPosition(0) * (7/150) / 2048 * 360));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        optimize(state, getTurningEncoderRadians()).speedMetersPerSecond;

    m_driveMotor.set(ControlMode.PercentOutput, driveOutput);
    m_turningMotor.set(ControlMode.Position, getTurnPulses(optimize(state, getTurningEncoderRadians()).angle.getRadians()));
  }

  public void configMotorPID(TalonFX talon, int slotIdx, double p, double i, double d){
    talon.config_kP(slotIdx, p);
    talon.config_kI(slotIdx, i);
    talon.config_kD(slotIdx, d);
  }

//Zeros all the SwerveModule encoders.
  public void resetEncoders() {
    m_driveMotor.setSelectedSensorPosition(0);
    m_turningMotor.setSelectedSensorPosition(0);
  }

private void checkCanCoderMotorCoder(){
 if (resetIterations <1){
   resetIterations ++;
   System.out.println("bruh");
   m_turningEncoder.setPositionToAbsolute();
  
 }
 resetIterations++;
 }

private SwerveModuleState optimize(SwerveModuleState st, double currentAngleRadians){
  double changeAngleRads = st.angle.getRadians() - currentAngleRadians;
  while(Math.abs(changeAngleRads) > Math.PI / 2){
  if(changeAngleRads > Math.PI / 2){
    st = new SwerveModuleState(-st.speedMetersPerSecond, new Rotation2d(st.angle.getRadians() - Math.PI));
  } else if(changeAngleRads < -Math.PI / 2){
    st = new SwerveModuleState(-st.speedMetersPerSecond, new Rotation2d(st.angle.getRadians() + Math.PI));
  }
  changeAngleRads = st.angle.getRadians() - currentAngleRadians;
}
  return st;
}
private double getTurnPulses(double referenceAngleRadians){
  return referenceAngleRadians * MK4IModuleConstants.i_kEncoderCPR / 2 / Math.PI;
}

}
