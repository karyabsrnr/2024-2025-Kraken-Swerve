// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.JoystickContainer;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.OperatorConstants;
//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driveJoy = new XboxController(0);
    private final XboxController opJoy = new XboxController(1);
    public JoystickContainer joyStick = new JoystickContainer(driveJoy, opJoy);
//Use to be Joystick not XboxController
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveJoy, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driveJoy, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

  //    public double getDriveJoy(int axis){
  //   double raw = driveJoy.getRawAxis(axis);
  //   return Math.abs(raw) < 0.1 ? 0.0 : raw;
  // }

  // public double getOpJoy(int axis){
  //   double raw = opJoy.getRawAxis(axis);
  //   return Math.abs(raw) < 0.1 ? 0.0 : raw;
  // }

  // public double getDriveJoyXR(){
  //   double raw = getDriveJoy(4);
  //   return raw; 
  // }


  // public double getDriveJoyXL(){
  //   double raw = getDriveJoy(0); //Verify axis
  //   return raw; 
  // }

  // public double getDriveJoyYL(){
  //   double raw = getDriveJoy(1);
  //   return raw; 
  // }

  // public double getDriveJoyYR(){
  //   double raw = getDriveJoy(5);
  //   return raw;
  // }
  // // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveJoy.getRawAxis(translationAxis), 
                () -> -driveJoy.getRawAxis(strafeAxis), 
                () -> -driveJoy.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }
   

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      

      //  joyStick.driveButton(1)
    // .onTrue(new InstantCommand(()->{
    //   s_Swerve.setDefaultCommand(
  
    //         new TeleopSwerve(
    //             s_Swerve, 
    //             () -> 1, 
    //             () -> 1, 
    //             () -> -driveJoy.getRawAxis(rotationAxis), 
    //             () -> robotCentric.getAsBoolean()
    //         )
    //     );
  
    // }));

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    }

  
  
  

   /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
