// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SerialPort;

public class SwerveSubsystem extends SubsystemBase {

    //create the front left swerve module
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    //create the front right swerve module
    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    //create the back left swerve module
    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    //create the back right swerve module
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    //create gyro using NavX
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    //creates a position at the origin facing front
    private final Pose2d poseThis = new Pose2d();

    //the state of the swerve modules
    private final SwerveModulePosition[] Position = {frontLeft.getPosition(), frontRight.getPosition(), 
                                                     backLeft.getPosition(), backRight.getPosition()};

    //Constructs a SwerveDrivePoseEstimator with default 
    //standard deviations for the model and vision measurements.
    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            new Rotation2d(0), Position, poseThis);

    // Create two new SimpleMotorFeedforwards (one right and one left)
    //with gains kS, kV, and kA from SysID characterization
    private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(DriveConstants.kSRight, DriveConstants.kVRight, DriveConstants.kARight);
    private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(DriveConstants.kSLeft, DriveConstants.kVLeft, DriveConstants.kALeft);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    //resets the gyro
    public void zeroHeading() {
        gyro.reset();
    }

    //returns heading of gyro
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    //returns the current rotation
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //returns estimated robot position in meters
    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }

    //resets the robot's position on the field
    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] state = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        odometer.resetPosition(getRotation2d(), state, pose);
    }

    //updates the robot's position on the field
    @Override
    public void periodic() {
        getChassisPitchError();

        SwerveModulePosition[] state = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        odometer.update(getRotation2d(), state);

    }

    //stops all modules
    public void stopModules() {

        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();

    }

    //sets the modules state
    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], feedforwardLeft);
        frontRight.setDesiredState(desiredStates[1], feedforwardRight);
        backLeft.setDesiredState(desiredStates[2], feedforwardLeft);
        backRight.setDesiredState(desiredStates[3], feedforwardRight);

    }

    //returns the gyro's pitch in degrees    
    public float getChassisPitch() {

        return gyro.getPitch();

    }

    //returns the current pitch of the chassis
    public float getChassisPitchError() {

        double pitch = (double)gyro.getPitch();
        pitch = pitch * Math.PI/180;
        double g = 1.0;
        double a = gyro.getWorldLinearAccelX();
        pitch = pitch - Math.asin(((Math.sin(pitch) * 2.0 * g * Math.cos(pitch) + a) / Math.sqrt(Math.pow(g, 2.0) + Math.pow(Math.sin(pitch), 2.0) + 2.0 * g * Math.sin(pitch) + Math.pow(g, 2.0) * Math.pow(Math.cos(pitch), 2.0)))*(180.0/Math.PI));
        return (float)pitch;
    }
    
}
