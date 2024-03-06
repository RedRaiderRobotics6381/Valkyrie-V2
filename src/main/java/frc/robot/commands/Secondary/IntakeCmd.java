// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LauncherConstants;
//import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;

public class IntakeCmd extends Command {

  private LauncherRotateSubsystem launcherRotateSubsystem;
  private IntakeSubsystem intakeSubsystem;
  
  public IntakeCmd(IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherRotateSubsystem = launcherRotateSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);//, launcherRotateSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.hasNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherRotateSubsystem.launcherRotatePosCmd(LauncherConstants.posIntake);
    intakeSubsystem.intakeIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDsSubSystem.setLED(.91);
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.hasNote;
  }
}