// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;

public class LauncherRotateCmd extends Command{
  
  public static boolean speedComplete = false;
  private final LauncherRotateSubsystem m_launcherRotateSubsystem;
  private double launcherRotateSetpoint;

  
  /** Creates a new LauncherRotateCmd. */
  public LauncherRotateCmd(double launcherRotateSetpoint, LauncherRotateSubsystem launcherRotateSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherRotateSetpoint = launcherRotateSetpoint;
    this.m_launcherRotateSubsystem = launcherRotateSubsystem;
    addRequirements(launcherRotateSubsystem);
    
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    
    m_launcherRotateSubsystem.launcherRotatePosCmd(launcherRotateSetpoint);
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
    return m_launcherRotateSubsystem.launcherRotateComplete;
  }
}
