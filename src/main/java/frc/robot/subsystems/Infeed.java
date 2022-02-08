// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
  /** Creates a new Infeed. */
  private TalonSRX infeedMotor;

  private static Infeed _instance;
  public static final Infeed get_instance(){
    if (_instance == null){
      _instance = new Infeed();
    }
    return _instance;
  }

  public Infeed() {
    infeedMotor = new TalonSRX(8);
  }
  public void runInfeedMotor(double percentoutput){
    infeedMotor.set(ControlMode.PercentOutput, percentoutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
