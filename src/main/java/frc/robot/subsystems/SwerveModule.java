// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    //each module will have a turning and drive motor
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    //each module will be using its built in encoder
    //for their respective motor
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    //each module will have its PID controller built in
    private final PIDController turningPidController;

    //each module will have a CANCoder keeping
    //its true direction
    private final CANcoder absoluteEncoder;

    //whether the CANCoder needs to be inverted
    //and by how much it has to be offset in radians
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed,
                        boolean turningMotorReversed, int absoluteEncoderId, 
                        double absoluteEncoderOffset, boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    //returns number of rotations of the drive motor
    public double getDrivePosition() {

        return driveEncoder.getPosition();

    }

    //returns number of rotations of the turning motor
    public double getTurningPosition() {

        return turningEncoder.getPosition();

    }

    //returns the RPM of the drive motor
    public double getDriveVelocity() {

        return driveEncoder.getVelocity();

    }

    //returns the RPM of the turning motor
    public double getTurningVelocity() {

        return turningEncoder.getVelocity();

    }

    //calculates and returns the absolute encoder's
    //angle in radians
    public double getAbsoluteEncoderRad() {

        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI/180.0;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);

    }

    //resets the drive and turning encoders
    public void resetEncoders() {

        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());

    }

    //returns the position of the swerve module
    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));

    }

    //returns the speed of the swerve module
    public SwerveModuleState getState() {

        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));

    }

    //sets the desired state for each module
    public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward feedforward) {

        if (Math.abs(feedforward.calculate(state.speedMetersPerSecond)) < 0.1) {

            stop();
            return;

        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        
        }

    //stops all motors
    public void stop() {

        driveMotor.set(0);
        turningMotor.set(0);

    }
    
}
