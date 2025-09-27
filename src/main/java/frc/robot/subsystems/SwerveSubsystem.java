// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
   
  //making module locations
    private final Translation2d frontLeftLocation = new Translation2d(0.3429,0.3429);
    private final Translation2d frontRightLocation = new Translation2d(0.3429,-0.3429);
    private final Translation2d backLeftLocation = new Translation2d(-0.3429,0.3429);
    private final Translation2d backRightLocation = new Translation2d(-0.3429,-0.3429);
  


  
}