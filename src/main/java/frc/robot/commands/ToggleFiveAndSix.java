// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestingEther;

public class ToggleFiveAndSix extends CommandBase {
  private TestingEther _TEther = TestingEther.get_instance();
  /** Creates a new ToggleFiveAndSix. */
  public ToggleFiveAndSix() {
  addRequirements(_TEther);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _TEther.runFiveSix();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _TEther.runFiveSix();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _TEther.stopFiveSix();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
