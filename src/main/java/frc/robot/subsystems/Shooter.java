// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX _frontMotor;
  private TalonFX _backMotor;
  private double shooterFrontVbus = .50;
  private double shooterBackVbus = 0.7;

  private static Shooter _instance;
  public static Shooter get_instance(){
    if(_instance ==null){
      _instance = new Shooter();
    }
    return _instance;
  }
  public Shooter() {
    _frontMotor = new TalonFX(SubsystemConstants.SHOOTER_FRONT_MOTOR_ID);
    _backMotor = new TalonFX(SubsystemConstants.SHOOTER_BACK_MOTOR_ID);
  
    _backMotor.setInverted(InvertType.InvertMotorOutput);
    
  }

  public void runShooterMotors(){
    _frontMotor.set(ControlMode.PercentOutput, shooterFrontVbus);
    _backMotor.set(ControlMode.PercentOutput, shooterBackVbus);//.67);
  }

  public void stopShooterMotors(){
    _frontMotor.set(ControlMode.PercentOutput, 0);
    _backMotor.set(ControlMode.PercentOutput, 0);
  }

  public void shiftShooterVbus(double frontshift, double backshift){
    shooterBackVbus += backshift;
    shooterFrontVbus += frontshift;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
