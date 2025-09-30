// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

public class SwerveSubsystem extends SubsystemBase {

  // some constants
  private static final double maxSpeed = 4.47; // meters per second, theoretical max speed
  private static final double maxRotationSpeed = 2*Math.PI;
  private static final boolean gyroReversed = false; 

   
  // making module locations
  private final Translation2d frontLeftLocation = new Translation2d(0.3429,0.3429);
  private final Translation2d frontRightLocation = new Translation2d(0.3429,-0.3429);
  private final Translation2d backLeftLocation = new Translation2d(-0.3429,0.3429);
  private final Translation2d backRightLocation = new Translation2d(-0.3429,-0.3429);

  // make modules and assign them id's
  private final SwerveModule frontLeftModule = new SwerveModule(1, 2, 0);
  private final SwerveModule frontRightModule = new SwerveModule(3,4,1);
  private final SwerveModule backLeftModule = new SwerveModule(5, 6, 2);
  private final SwerveModule backRightModule = new SwerveModule(7, 8, 3);

  private final AHRS gyro = new AHRS(NavXComType.kI2C, NavXUpdateRate.k200Hz);

  // set up swerve kinematics
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  // set up odometry
  private final SwerveDriveOdometry odometry = 
  new SwerveDriveOdometry(kinematics, 
  Rotation2d.fromDegrees(gyro.getAngle()),
  new SwerveModulePosition[]{
    frontLeftModule.getPosition(),
    frontRightModule.getPosition(),
    backLeftModule.getPosition(),
    backRightModule.getPosition()
  });

  // reset gyro on start up
  public void Drivetrain(){
    gyro.reset();
  }

  // update the odometry periodically
  public void periodic() {
    odometry.update(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      new SwerveModulePosition[]{
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    });
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // to reset odometry
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
      Rotation2d.fromDegrees(gyro.getAngle()), 
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(),
        backRightModule.getPosition()
    },
    pose);
  }

/**
 * 
 * @param vx                speed of robot in the X direction (forward and back)
 * @param vy                speed of robot in the Y direction (left to right)
 * @param rot               Angular rate
 * @param fieldRelative     Whether x and y speeds are relative to the field or nah
 */

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedMeters = xSpeed * maxSpeed;
    double ySpeedMeters = ySpeed * maxSpeed;
    double rotRadians = rot * maxRotationSpeed;

    var moduleStates = kinematics.toSwerveModuleStates(
    fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMeters, ySpeedMeters, rotRadians, 
        Rotation2d.fromDegrees(gyro.getAngle()))
        : new ChassisSpeeds(xSpeedMeters, ySpeedMeters, rotRadians));

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

    // inteded to fully stop movement, not on a controller
  public void fullStop() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  public void resetHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate() * (gyroReversed ? -1:1);
  }
}