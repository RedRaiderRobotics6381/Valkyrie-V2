// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ClimberSubsystem;


public class ClimberLowerCmd extends Command {

  private ClimberSubsystem climberSubsystem;

  public ClimberLowerCmd(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.lowered = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSubsystem.climberLower();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.climberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.lowered;
  }
}