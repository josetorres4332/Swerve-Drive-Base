// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {

        //converts the wheel diameter to meters
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        //gear ratio for the MK4i drive motor (comes from Swerve Specialties Website)
        public static final double kDriveMotorGearRatio = 1/6.75;

        //gear ratio for the MK4i turning motor (comes from Swerve Specialties Website)
        public static final double kTurningMotorGearRatio = 7.00 / 150.00;

        //converts rotation of drive wheel to meters
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio *  kWheelDiameterMeters * Math.PI;
        
        //convert rotation of turning wheel to radians
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;

        //rotations per second of drive wheel
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;

        //rotation per second of turning wheel
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;

        //PID Controller P turning value
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        // Distance between right and left module
        public static final double kTrackWidth = Units.inchesToMeters(29.875);
        
        // Distance between front and back module
        public static final double kWheelBase = Units.inchesToMeters(29.875);

        /*Translation2d locates each module based on the robot's center by
         *creating a coordinate (X, Y) for each module 
         *
         *                        FRONT OF ROBOT
         * 
         *                            WIDTH
         *                ______________________________
         *                | LEFT        |       RIGHT  |
         *           L    | FRONT       |       FRONT  |
         *           E    |             |              |
         *           N    |             |              |
         *           G    |_____________|______________|
         *           T    |             |              |
         *           H    |             |              |
         *                | LEFT        |       RIGHT  |  
         *                | REAR        |       REAR   |
         *                |_____________|______________| 
         */

        /*SwerveDriveKinematics uses the information from Translation 2d to
         *calculate the vectors (velocity and angle) for each swerve drive module */       
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), /* Coordinate of the Right Front module */
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), /* Coordinate of the Left Front module */
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), /* Coordinate of the Right Rear module */
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /* Coordinate of the Left Rear module */
        
        //CAN IDs for all drive and turning motors
        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 2;
        public static final int kFrontLeftTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 1;

        //whether the turning encoders are reversed or not
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        //whether the drive encoders are reversed or not
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //CAN IDs for the CANCoders
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 6;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 9;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        //whether the CANCoders are reversed or not
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //sets the offset for each CANCoder in radians
        //value must be given is degrees
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0 * Math.PI/180.0;

        //MAX speed in meters per second of drive motor
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

        //MAX speed in radians per second of turning motor
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //MAX speed and turning velocities
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        //Feedforward constants from SysID
        //UPDATED ON: 
        public static final double kSLeft = 0.097576;
        public static final double kVLeft = 2.6933;
        public static final double kALeft = 0.26236;

        public static final double kSRight = 0.099437;
        public static final double kVRight = 2.6173;
        public static final double kARight = 0.11195;

        public static final double kRotGain = 2;
        public static final double kDriveGain = 3;

    }

    public static final class AutoConstants {

        //MAX speeds of modules in Autonomous
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    
    }

    public static final class OIConstants {

        //Driver joystick controller ID
        public static final int kDriverControllerPort = 0;

        /************************Joystick Axis and Buttons ************************
         *                                  AXISES
         * Left X Stick LEFT/RIGHT = 0                     Left Y Stick UP/DOWN = 1
         * Left Trigger = 2                                Right Trigger = 3
         * Right X Stick LEFT/RIGHT = 4                    Right Y Stick LEFT/RIGHT = 5 
         *                                  BUTTONS
         * Button A = 0                                    Button B = 1
         * Button X = 2                                    Button Y = 3
         * Left Bumper = 4                                 Right Bumper = 5
         * Pause Button = 6                                Menu Button = 7  
         */

        //Moves robot FORWARD and BACK using
        //Left Y Stick UP/DOWN
        public static final int kDriverYAxis = 1;

        //Moves robot LEFT and RIGHT using
        //Right X Stick LEFT/RIGHT
        public static final int kDriverXAxis = 4;

        //Rotates robot LEFT using
        //Left Trigger
        public static final int kDriverLeftRotAxis = 2;
        
        //Rotates robot RIGHT using
        //Right Trigger
        public static final int kDriverRightRotAxis = 3;

        //button selects whether the robot is
        //Field Centric or Robot Centric
        public static final int kDriverFieldOrientedButtonIdx = 5;

        //driver reset gyro button
        public static final int kDriverResetGyroButtonIdx = 2;

        //kills any drift using this deadband
        public static final double kDeadband = 0.1;        

    }

    public static class OperatorConstants {

        //Operator joystick controller ID
        public static final int kOperatorControllerPort = 1;

        /************************Joystick Axis and Buttons ************************
         *                                  AXISES
         * Left Stick LEFT/RIGHT = 0                       Left Stick UP/DOWN = 1
         * Left Trigger = 2                                Right Trigger = 3
         * Right Stick LEFT/RIGHT = 4                      Right Stick LEFT/RIGHT = 5 
         *                                  BUTTONS
         * Button A = 0                                    Button B = 1
         * Button X = 2                                    Button Y = 3
         * Left Bumper = 4                                 Right Bumper = 5
         * Pause Button = 6                                Menu Button = 7  
         */

         //Operator reset gyro button
         public static int kOperatorResetGyroButtonIdx = 3;

    }
    
    public static final class BalanceConstants {

        //uses these PID values for balancing
        public static final double balanceP = 0.0275;
        public static final double balanceI = 0;
        public static final double balanceD = .01;

    }

}