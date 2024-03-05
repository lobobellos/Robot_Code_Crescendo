// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final class CurrentLimits {
		public static final int driveMotorStall = 30;
		public static final int driveMotorFree = 35;

		public static final int turningMotorStall = 30;
		public static final int turningMotorFree = 35;
	}

	public static final class DriveConstants {
		public static final int kFrontLeftDriveMotorPort = 7;
		public static final int kRearLeftDriveMotorPort = 5;
		public static final int kFrontRightDriveMotorPort = 1;
		public static final int kRearRightDriveMotorPort = 3;

		public static final int kFrontLeftTurningMotorPort = 8;
		public static final int kRearLeftTurningMotorPort = 6;
		public static final int kFrontRightTurningMotorPort = 2;
		public static final int kRearRightTurningMotorPort = 4;

		public static final int kFrontLeftTurningEncoderPorts = 2;
		public static final int kRearLeftTurningEncoderPorts = 3;
		public static final int kFrontRightTurningEncoderPorts = 1;
		public static final int kRearRightTurningEncoderPorts = 0;

		public static final double kFrontLeftTurningEncoderOffset = -0.063; // 8
		public static final double kRearLeftTurningEncoderOffset = -0.048; // 6
		public static final double kFrontRightTurningEncoderOffset = -0.885;// 2
		public static final double kRearRightTurningEncoderOffset = -0.338;// 4

		public static final boolean kFrontLeftDriveEncoderReversed = false;
		public static final boolean kRearLeftDriveEncoderReversed = true;
		public static final boolean kFrontRightDriveEncoderReversed = false;
		public static final boolean kRearRightDriveEncoderReversed = false;

		public static final double kTrackWidth = 0.65;
		// Distance between centers of right and left wheels on robot
		public static final double kWheelBase = 0.45;
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final boolean kGyroReversed = false;

		public static final double kMaxSpeedMetersPerSecond = 4.2;
		public static final double kDemoSpeedMetersPerSecond = 1.0;

		// used for keeping the robot pointed in the right direction
		public static final class RotationPID {
			public static final double kP = 4.5;
			public static final double kI = 0.0;
			public static final double kD = 0.2;
		}

		public static final class RotationFF {
			public static final double kS = 0.05;
			public static final double kV = 0.05;
			public static final double kA = 0.0;
		}

		public static final double rotationPostitionTolerance = 2.0;
		public static final double rotationVelocityTolerance = 0.0002;
	}

	public static final class ModuleConstants {

		public static final class TurningPID {
			public static double kP = 0.8;
			public static double kI = 0.0;
			public static double kD = 0.0;
		}

		public static final class DrivePID {
			public static double kFF = 0.22;
			public static double kP = 0.1;
			public static double kI = 0.0;
			public static double kD = 0.05;
		}

		public static final double kMaxModuleAngularSpeed = 100; // rad/s
		public static final double kMaxModuleAngularAcceleration = 100; // rad/s^2

		public static final double kWheelDiameter = 0.095; // meters
		public static final double driveGearRatio = 1.0 / 6.75;

		// converting to m
		public static final double kdrivePositionConversionFactor = driveGearRatio * kWheelDiameter * Math.PI;
		// converting to m/s
		public static final double kdriveVelocityConversionFactor = (driveGearRatio / 60.0) * kWheelDiameter * Math.PI;
		// converting to radians
		public static final double kTurningConversionFactor = 1.0 / 12.8;
	}

	public static final class PDPConstants {
		public static final int deviceID = 10;
	}

	public static final class PneumaticsConstants {
		public static final int kPneumaticHubPort = 11;

		public static final double maxPressure = 110; // PSI

		public static final int pistonDownPort = 0;
		public static final int pistonUpPort = 1;
	}

	public static final class IntakeConstants {
		public static final int topMotorID = 12;
		public static final int BottomMotorID = 13;

		public static final boolean kTopInverted = false;
		public static final boolean kBottomInverted = true;

		public static final double kSpeedPercent = 0.5;

		public static final int freeCurentLimit = 25;
		public static final int stallCurentLimit = 20;
	}

	public static final class ElevatorConstants {
		public static final int topMotorID = 14;
		public static final int BottomMotorID = 15;

		public static final boolean kTopInverted = true;
		public static final boolean kBottomInverted = false;

		public static final double kSpeedPercent = 0.5;

		public static final int freeCurentLimit = 20;
		public static final int stallCurentLimit = 10;
	}

	public static final class HookConstants {
		public static final int kHookMotorCANID = 18;
		public static final int limitSwitchTopPort = 0;
		public static final int limitSwitchBottomPort = 1;

		public static final double deploySpeed = 0.5;
		public static final double retractSpeed = 0.5;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kMechanismControllerPort = 1;

		public static final double joystickDeadband = 0.15;

		public static final double rotationMultiplier = 0.01;
	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;

		// Constraint for the motion profiled robot angle controller
		public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
				new PIDConstants(0.5, 0, 0, 0), // translation constants
				new PIDConstants(0, 0, 0.0), // rotation constants
				AutoConstants.kMaxSpeedMetersPerSecond, // max velocity
				Math.hypot(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2),
				new ReplanningConfig());
	}

	public static final class AutoAlignmentConstants {
		public static double positionSetpointX = 0;
		public static double positionSetpointY = -0.35;
		public static double positionTolerance = 0.02;

		public static Rotation2d rotationSetpoint = Rotation2d.fromRotations(0);
		public static Rotation2d rotationTolerance = Rotation2d.fromDegrees(2);

		public static final class MovementPID {
			public static final double kP = 1.0;
			public static final double kI = 0.0;
			public static final double kD = 0.0;
		}

		public static final class MovementFF {
			public static final double kS = 0.1;
			public static final double kV = 0.0;
			public static final double kA = 0.0;
		}

		public static final class RotationPID {
			public static final double kP = 0.06;
			public static final double kI = 0.0;
			public static final double kD = 0.002;
		}

		public static final class RotationFF {
			public static final double kS = 0.12;
			public static final double kV = 0.000;
			public static final double kA = 0.0;
		}
	}

	public static final class LedConstants {
		public static final int numLeds = 200;

		public static final int PWMPort = 0;

		// for configuring the display
		public static final int hue = 319;
		public static final int saturation = (int) 0.69 * 255;
		public static final double valueBase = 40;

		public static final double valuePeriod = 0.2;
		public static final double valueFrequency = valuePeriod / 1;
		public static final double valueAmplitude = 2;

	}

	public static final class LimelightConstants {
		public static final double defaultReturnValue = -1;
		public static final double alignmentTolerance = 5;

		public static final double targetHeight = 0.7;
		public static final double targetWidth = 0.5;
		public static final double targetFloorHeight = 0.0;

		public static final double mountingAngleDegrees = 0.0;
		public static final double mountingHeightInches = 0.0;
	}

	public static final class ShooterConstants {
		public static final int topMotorID = 17;
		public static final int bottomMotorID = 16;

		public static final boolean kTopInverted = true;
		public static final boolean kBottomInverted = true;

		// speeds in RPM
		public static final double ampTopMotorSpeed = 725;
		public static final double ampBottomMotorSpeed = 775;

		public static final double speakerTopMotorSpeed = 1800;
		public static final double speakerBottomMotorSpeed = 1800;

		public static final double speedTolerance = 15;

		public static final int freeCurentLimit = 25;
		public static final int stallCurentLimit = 20;

		public static final class FlywheelPIDF {
			public static final double kP = 0.000125;
			public static final double kI = 0.0;
			public static final double kD = 8e-7;
			public static final double kF = 0.00022;
		}
	}
}
// top, bottom
// 400, 400 : nowhere near enough power
// 600, 600 : wheels added no discernable difference
// 800, 800 : rather high but went in
// 700, 700 : in but funky
// 700, 700 : in but funky
// 700, 700 : in but funky
// 700, 700 : in but funky
// 650, 700 : easy in

//xoxooxxooooxxxx|xxxxx