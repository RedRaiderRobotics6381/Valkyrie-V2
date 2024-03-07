// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LauncherRotateCmd extends WaitCommand{
  
  public static boolean speedComplete = false;
  private LauncherRotateSubsystem launcherRotateSubsystem;
  private double launcherRotateSetpoint;

  
  /** Creates a new LauncherRotateCmd. */
  public LauncherRotateCmd(double launcherRotateSetpoint, LauncherRotateSubsystem launcherRotateSubsystem) {
    super(0.25);
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherRotateSetpoint = launcherRotateSetpoint;
    this.launcherRotateSubsystem = launcherRotateSubsystem;
    addRequirements(launcherRotateSubsystem);
    
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    
    launcherRotateSubsystem.launcherRotatePosCmd(launcherRotateSetpoint);
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return launcherRotateSubsystem.launcherRotateComplete;
  }
}
