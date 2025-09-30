package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;


public class SwerveModule {
    
    // module constants
    private static final double wheelRadius = 0.0508;
    private static final double driveGearRatio = 6.75;
    private static final double moduleMaxAngularVelocity = 27.7;        // estimations
    private static final double moduleMaxAngularAcceleration = 556.6;        // estimations


    // make each motor per module
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    // make encoder for motor
    private final AnalogEncoder steerEncoder;
 

    // set pid gains
    private final PIDController drivePIDController =  new PIDController(1,0,.67);

    private final ProfiledPIDController steerPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              moduleMaxAngularVelocity, moduleMaxAngularAcceleration));

    // set feedforward
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0,0,0);
    
    // creates module 
    public SwerveModule(int driveID, int steerID, int steerChannel) {
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);

        steerEncoder = new AnalogEncoder(steerChannel);
        
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    // get current module state

    public SwerveModuleState getState() {
        // get wheel speed in rpm then converts it into mps
        double motorRPM = driveMotor.getEncoder().getVelocity();        
        double wheelSpeed = motorRPM                                    // RPM
                            / 60                                        // RPS
                            / driveGearRatio                            // something
                            * (2*Math.PI + wheelRadius);                 // meters cuz circumfrence

        return new SwerveModuleState(
        wheelSpeed, new Rotation2d(steerEncoder.get())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
        driveMotor.getEncoder().getPosition(),
        new Rotation2d(steerEncoder.get()*2*Math.PI)            // encoder rotations to radians
        );
    }
        
    // ts will NOT work
    public void setDesiredState(SwerveModuleState desiredState) {
        
        // so tired so i made it twice
        double motorRPM = driveMotor.getEncoder().getVelocity(); 
        double wheelSpeed = motorRPM                                   // RPM
                            / 60                                        // RPS
                            / driveGearRatio                            // something
                            * (2*Math.PI + wheelRadius);                 // meters cuz circumfrence


        double driveOutput = drivePIDController.calculate(wheelSpeed, desiredState.speedMetersPerSecond);
        double driveFF = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        driveMotor.setVoltage(driveOutput+driveFF);

        double steerOutput = steerPIDController.calculate(getState().angle.getRadians(), desiredState.angle.getRadians());
        steerMotor.set(steerOutput);
    
  }

  // Zeroes all the SwerveModule encoders.
  public void resetEncoders() {
    driveMotor.getEncoder().setPosition(0);
  }
}
