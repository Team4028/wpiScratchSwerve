// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class TestingEther extends SubsystemBase {
  /** Creates a new TestingEther. */
  private TalonSRX _infeedMotor;
  private CANSparkMax _singulatorMotor;
  private CANSparkMax _conveyorMotor; // needs to be reversed

  private TalonFX _MotorFive;
  private TalonFX _MotorSix;

  private RelativeEncoder _enc;
  private static TestingEther _instance = new TestingEther();
 public static TestingEther get_instance() {
   return _instance;
 }


  public TestingEther() {
  _infeedMotor = new TalonSRX(SubsystemConstants.INFEED_MOTOR_ID);
  _singulatorMotor = new CANSparkMax(SubsystemConstants.SINGULATOR_MOTOR_ID, MotorType.kBrushless);
  _conveyorMotor = new CANSparkMax(SubsystemConstants.CONVEYOR_MOTOR_ID, MotorType.kBrushless);

  _MotorFive = new TalonFX(5);
  _MotorSix = new TalonFX(6);

    _enc = _conveyorMotor.getEncoder();
    _enc.setPosition(0);

    _conveyorMotor.setInverted(true);
 
  }

  public void runMotorOneAndTwo(){
    _infeedMotor.set(ControlMode.PercentOutput, .6);
    _singulatorMotor.set(.6);
  }

  public void runThree(){
    _conveyorMotor.set(0.5);
  }

  public void stopThree(){
    _conveyorMotor.set(0);
  }

  public void stopTwoThree(){
    _infeedMotor.set(ControlMode.PercentOutput, 0);
    _singulatorMotor.set(0);
    _conveyorMotor.set(0);
  }

  public void runMotorWithEncoder(double target, double vbus){
    if (_enc.getPosition()<target)
    {
      _conveyorMotor.set(0);
    }
    else{
      _conveyorMotor.set(vbus);
    }
    
  }

  

  

  public void runFiveSix(){
    _MotorFive.set(ControlMode.PercentOutput, .4);
    _MotorSix.set(ControlMode.PercentOutput, .6);
  }

  public void stopFiveSix(){
    _MotorFive.set(ControlMode.PercentOutput, 0);
    _MotorSix.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
