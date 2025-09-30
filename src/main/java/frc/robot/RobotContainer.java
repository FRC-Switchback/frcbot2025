// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.CoralStackCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  private final CommandXboxController operatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public final RollerSubsystem m_roller = new RollerSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public final SwerveSubsystem m_drive = new SwerveSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final SwerveSubsystem swerveDrive = new SwerveSubsystem();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    configureBindings();
    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    SmartDashboard.putData(m_chooser);

    swerveDrive.setDefaultCommand(

        new RunCommand(
          () -> swerveDrive.drive(
            -MathUtil.applyDeadband(driverController.getLeftX(), 0.05),      // second value is the deadband
            -MathUtil.applyDeadband(driverController.getLeftY(), 0.05),
            -MathUtil.applyDeadband(driverController.getRightX(), 0.05),
            true
          ),
          swerveDrive));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      JoystickButton(driverController, XboxController.Button.kLeftStick)
              .whileTrue(new RunCommand(
                  () -> swerveDrive.fullStop(),
                  swerveDrive));
      
      
          /**
           * Here we declare all of our operator commands, these commands could have been
           * written in a more compact manner but are left verbose so the intent is clear.
           */
          operatorController.rightBumper().whileTrue(new AlgieInCommand(m_roller));
          
          // Here we use a trigger as a button when it is pushed past a certain threshold
          operatorController.rightTrigger(.2).whileTrue(new AlgieOutCommand(m_roller));
      
          /**
           * The arm will be passively held up or down after this is used,
           * make sure not to run the arm too long or it may get upset!
           */
          operatorController.leftBumper().whileTrue(new ArmUpCommand(m_arm));
          operatorController.leftTrigger(.2).whileTrue(new ArmDownCommand(m_arm));
      
          /**
           * Used to score coral, the stack command is for when there is already coral
           * in L1 where you are trying to score. The numbers may need to be tuned, 
           * make sure the rollers do not wear on the plastic basket.
           */
          operatorController.x().whileTrue(new CoralOutCommand(m_roller));
          operatorController.y().whileTrue(new CoralStackCommand(m_roller));
      
          /**
           * POV is a direction on the D-Pad or directional arrow pad of the controller,
           * the direction of this will be different depending on how your winch is wound
           */
          operatorController.pov(0).whileTrue(new ClimberUpCommand(m_climber));
          operatorController.pov(180).whileTrue(new ClimberDownCommand(m_climber));
        }
      
        private Trigger JoystickButton(CommandXboxController driverController2, Button kleftstick) {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'JoystickButton'");
        }
      
      
        /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }
}