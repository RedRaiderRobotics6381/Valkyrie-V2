// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (141.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.1, 0, 0.1);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class Drivebase
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10;
    public static double Max_Speed_Multiplier = 0.5;
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.025;
    public static final double LEFT_Y_DEADBAND  = 0.025;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class ClimberConstants
  {
    public static final int kClimberMotorL = 20;
    public static final int kClimberMotorR = 21;
  }
  
  public static final class LauncherConstants
  {
    public static final int kLauncherT = 18;
    public static final int kLauncherB = 19;
    public static final int kLauncherRotate = 16;

    public static final double launcherMotorTopSpeed = 0.8;
    public static final double launcherIndexerMotorSpeed = 1;
    public static final double posIntake = 124.5;

    public static double TrapScoreAngle = 145.0; //from 147.5
    public static double TrapScoreSpeed = 2000; //from 712.5
    public static double SpeakerScoreAngle = 153.5;
    public static double SpeakerScoreSpeed = 2200; //from 2000
    public static double AmpScoreAngle = 153.5;
    public static double AmpScoreSpeed = 1000; //from 800
    public static double kAutoScoreSpeed = 1750; //from 800
    public static double kAutoScoreSpeedMin = 1750; //from 800
    public static double kAutoScoreSpeedMax = 4000; //from 800
    public static double kAutoScoreAimHeight = 1.808;
    public static double kAutoScoreAimAngle = 153.5;
    public static double LauncherSpeedMult;
    public static double LauncherSpeedTol = 10.0;
    public static double LauncherAngleTol = 1.0;

  }

  public static final class AprilTagConstants
  {
    public static int ampID = 0;
    public static int speakerID = 0;
    public static int stageIDA = 0;
    public static int stageIDB = 0;
    public static int stageIDC = 0;
  }

  public static final class IntakeConstants
  {
    public static final int kIntakeMotor = 14;
    public static final int kIndexerMotor = 15;
    public static final int kLauncherIndexerMotor = 17;
    public static final double intakeSpeed = 1;
    public static final double zeroSpeed = 0;
    public static final double indexerSpeed = 0.1;
    public static final double launcherIndexerSpeed = 0.12;
    public static final double indexerOuttakeSpeed = 1.0;
    public static final double launcherIndexerOuttakeSpeed = 1.0;
    public static final double indexerIntakeSpeed = 0.1;
    public static final double launcherIndexerIntakeSpeed = 0.1;
  }

}
