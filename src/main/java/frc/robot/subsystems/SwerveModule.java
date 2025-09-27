package frc.robot.subsystems;

import frc.robot.subsystems.SwerveSubsystem;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SwerveModule {
    
    // module constants
    private static final double wheelRadius = 0.0508;
    private static final double encoderResolution = 4096;

    private static final double moduleMaxAngularVelocity = ;
    private static final double moduleMaxAngularAcceleration = ;

    // make each motor per module
    private final PWMSparkMax driveMotor;
    private final PWMSparkMax steerMotor;

    // make encoder for motor
    private final AnalogEncoder steerEncoder;
    private final SparkMax driveMotorSparkMax;

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
    private final SimpleMotorFeedforward steerFeedforward = new SimpleMotorFeedforward(0,0,0);
    
    // creates module and connects motors to info channels
    public SwerveModule(
    int driveMotorChannel,
    int steerMotorChannel,
    int steerEncoderChannel
    ) {
         driveMotor = new PWMSparkMax(driveMotorChannel);
         steerMotor = new PWMSparkMax(steerMotorChannel);

         
         driveMotorSparkMax = new driveMotorSparkMax.get();
         steerEncoder = new AnalogEncoder(steerEncoderChannel);

         steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    // get current module state

    public SwerveModuleState getState() {
        return new SwerveModuleState(
        SparkMax.get(), new Rotation2d(steerEncoder.get())
        );
    }
    
}
