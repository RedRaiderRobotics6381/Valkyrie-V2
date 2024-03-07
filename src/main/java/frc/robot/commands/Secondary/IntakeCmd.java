// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Secondary;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.LauncherConstants;
// //import frc.robot.Constants.LauncherConstants;
// import frc.robot.subsystems.Secondary.IntakeSubsystem;
// import frc.robot.subsystems.Secondary.LEDsSubSystem;
// import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;

// public class IntakeCmd extends Command {

//   private final LauncherRotateSubsystem m_launcherRotateSubsystem;
//   private final IntakeSubsystem m_intakeSubsystem;
  
//   public IntakeCmd(IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem){
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.m_launcherRotateSubsystem = launcherRotateSubsystem;
//     this.m_intakeSubsystem = intakeSubsystem;
//     addRequirements(m_intakeSubsystem, m_launcherRotateSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_intakeSubsystem.hasNote = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_launcherRotateSubsystem.launcherRotatePosCmd(LauncherConstants.posIntake);
//     m_intakeSubsystem.intakeIntake();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     LEDsSubSystem.setLED(.91);
//     m_intakeSubsystem.stopIntake();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return m_intakeSubsystem.hasNote;
//   }
// }