// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command{

  //instance of needed objects
  private final SwerveSubsystem swerveSubsystem; 
  private final Supplier<Double> xSpdFunction, ySpdFunction, leftturningSpdFunction, rightturningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;


  /** Creates a new SwerveJoystickCmd.
   *  - swerveSubystem in brought in
   *  - joystick X value is brought in
   *  - joystick Y value is brought in
   *  - joystick LEFT TRIGGER is brought in
   *  - joystick RIGHT TRIGGER is brought in
   *  - whether field orientation is TRUE or FALSE is brought in  */
  public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,  Supplier<Double> xSpdFunction,
                           Supplier<Double> ySpdFunction, Supplier<Double> leftturningSpdFunction,
                           Supplier<Double> rightturningSpdFunction, Supplier<Boolean> fieldOrientedFunction){
    
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.leftturningSpdFunction = leftturningSpdFunction;
        this.rightturningSpdFunction = rightturningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // 1. Get real-time joystick inputs
    double xSpeed = -xSpdFunction.get();
    double ySpeed = -ySpdFunction.get();

    //1.1 this sets the trigger buttons so that
    //    they can be used for rotation of the robot
    double turningSpeed = 0;

    //1.1.a  if LEFT TRIGGER is pressed and RIGHT TRIGGER is not
    //       use value of LEFT TRIGGER
    if(leftturningSpdFunction.get() > 0 && rightturningSpdFunction.get() == 0){

      turningSpeed = leftturningSpdFunction.get();

    }

    //1.1.b  if RIGHT TRIGGER is pressed and LEFT TRIGGER is not
    //       use value of RIGHT TRIGGER inverted
    else if(rightturningSpdFunction.get() > 0 && leftturningSpdFunction.get() == 0){

      turningSpeed = -rightturningSpdFunction.get();

    }

    //1.1.c  if LEFT TRIGGER and RIGHT TRIGGER is pressed
    //       then the value is 0
    else if(leftturningSpdFunction.get() > 0 && rightturningSpdFunction.get() > 0){

      turningSpeed = 0;
    }

    //1.1.d  if LEFT TRIGGER and RIGHT TRIGGER is NOT pressed
    //       then the value is 0
    else{

      turningSpeed = 0;

    }    

    // 2. Apply deadband
    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? (xSpeed * Math.pow(Math.E, (DriveConstants.kDriveGain * Math.abs(xSpeed))))/Math.pow(Math.E, DriveConstants.kDriveGain) : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? (ySpeed * Math.pow(Math.E, (DriveConstants.kDriveGain * Math.abs(ySpeed))))/Math.pow(Math.E, DriveConstants.kDriveGain) : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? ((turningSpeed * Math.pow(Math.E, (DriveConstants.kRotGain * Math.abs(turningSpeed))))/Math.pow(Math.E, DriveConstants.kRotGain))/4 : 0.0;

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;

    //checks to see if Field Orientation is relative to field
    if (fieldOrientedFunction.get()) {
        
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } 

    //checks to see if Field Orientation is relative to robot    
    else {
        
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerveSubsystem.stopModules();

  }

  //Everything is finished
  @Override
  public boolean isFinished() {

    return false;

  }
  
}
