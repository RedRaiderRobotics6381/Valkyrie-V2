// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

//import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.LauncherRotateSubsystem;
import frc.robot.subsystems.Secondary.LauncherSubsystem;

public class ScoreManCmd extends SequentialCommandGroup {
  /** Creates a new Outtake. */
  public ScoreManCmd(double launcherAngle, double launcherSpeed, LauncherSubsystem launcherSubsystem, IntakeSubsystem intakeSubsystem, LauncherRotateSubsystem launcherRotateSubsystem) {
    if(Robot.sensorOuttake.get() == true || Robot.sensorIntake.get() == true){
      addCommands(
        new LauncherRotateCmd(launcherAngle, launcherRotateSubsystem),
        new LauncherSpeedCmd(launcherSpeed, launcherSubsystem),
        new WaitUntilCommand(() -> launcherRotateSubsystem.launcherRotateComplete()).withTimeout(2),
        new WaitUntilCommand(() -> launcherSubsystem.launcherSpeedComplete()).withTimeout(1),
        new OuttakeCmd(intakeSubsystem)
      );
    }
  }
}

