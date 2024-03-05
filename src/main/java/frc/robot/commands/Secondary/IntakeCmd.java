// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LEDsSubSystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;

public class IntakeCmd extends Command {

  private boolean hasNote = false;
  private final LauncherRotateSubsystem launcherRotateSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  
  public IntakeCmd(IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherRotateSubsystem = launcherRotateSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem, this.launcherRotateSubsystem);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.sensorOuttake.get() == true){
      hasNote = true;
    } else {
      new LauncherRotateCmd(LauncherConstants.posIntake, launcherRotateSubsystem);
      intakeSubsystem.indexerMotor.set(IntakeConstants.indexerIntakeSpeed);
      intakeSubsystem.intakeMotor.set(IntakeConstants.intakeSpeed);
      intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.launcherIndexerSpeed);
      //System.out.println(Robot.sensorIntake.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LEDsSubSystem.setLED(.71);
    intakeSubsystem.indexerMotor.set(IntakeConstants.zeroSpeed);
    intakeSubsystem.intakeMotor.set(IntakeConstants.zeroSpeed);
    intakeSubsystem.launcherIndexerMotor.set(IntakeConstants.zeroSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasNote;
  }
}