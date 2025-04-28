/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;

//import edu.wpi.first.wpilibj.Joystick;

import java.util.Map;

//import com.ctre.phoenix6.hardware.Pigeon2;
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS5Controller;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
//import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos.SimpleAuto;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.OperatorCommands.ControllerCommands.*;
import frc.robot.subsystems.ScoringSubsystem;
//import frc.robot.commands.OperatorCommands.JoyStickCommands.*;
//import frc.robot.commands.autoCommands.PIDButtons;
//import frc.robot.subsystems.PIDSubsystems.*;
import frc.robot.subsystems.SwerveSubsystems.SwerveSubsystem;
//import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
//import swervelib.parser.SwerveParser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  //private final SendableChooser<Command> m_chooser;
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer() {
    //m_chooser = AutoBuilder.buildAutoChooser();

    configureInitialDefaultCommands();
    configureBindings();
    configureShuffleboardData();
    configureSmartDashboard();

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  // The robot's subsystems and commands are defined here...
  /// SHUFFLEBOARD TAB ///
  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Competition Robot");

  /// SUBSYSTEMS ///
  /// 
File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
 private final SwerveSubsystem drivetrain = new SwerveSubsystem(swerveJsonDirectory);

  public static final ScoringSubsystem scoreSub = new ScoringSubsystem();
  //public static final ArmSubsystem armSub = new ArmSubsystem();
  //public static final ElevatorSubsystem yeetSub = new ElevatorSubsystem();

  /// OI DEVICES / HARDWARE ///
  private final XboxController xbox = new XboxController(0);
  //private final PS5Controller psCon = new PS5Controller(1);
  //private final Joystick stick = new Joystick(1);

  //private static final Pigeon2 gyro = new Pigeon2(Constants.pidgeonID);

  CommandXboxController commandController = new CommandXboxController(0);
  //CommandPS5Controller psCommandController = new CommandPS5Controller(1);

  //intake buttons
  //JoystickButton intakeFoward = new JoystickButton(stick, 2);
  //JoystickButton intakeOff = new JoystickButton(stick, 3);

  /// COMMANDS ///
  // Xbox controls
  private final DriveSwerve drivetrainXbox = new DriveSwerve(drivetrain, ()-> -xbox.getLeftY(), ()-> -xbox.getLeftX(), ()-> xbox.getRightX()); //,
    //()-> xbox.getXButton(), ()-> xbox.getYButton());  //, ()-> xbox.getXButtonPressed()); 
  //     low power                          high power                       forms X with wheels
  // getLeftBumperButtonPressed()
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivetrain.getSwerveDrive(),
    () -> commandController.getLeftY() * -1, () -> commandController.getLeftX() * -1)
    .withControllerRotationAxis(commandController::getRightX)
    .deadband(Constants.SPEED_DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

    /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(commandController::getRightX,
  commandController::getRightY).headingWhile(true);



  //private final DriveSwerve drivetrainXbox = new DriveSwerve(drivetrain, ()-> -xbox.getLeftX(), ()-> xbox.getLeftY(), ()-> -xbox.getRightX());


  // PS5 Controls
  //private final DriveSwerve drivePS = new DriveSwerve(drivetrain, ()-> -psCon.getLeftX(), ()-> -psCon.getLeftY() , ()-> -psCon.getRightX(),
    //()-> psCon.getL1ButtonPressed(), ()-> psCon.getR1ButtonPressed());  //, ()-> xbox.getXButtonPressed()); 
  //    RB low power                 LB high power                       forms X with wheels

  //private final DriveSwerve drivePS = new DriveSwerve(drivetrain, ()-> -psCon.getLeftX(), ()-> psCon.getLeftY(), ()-> -psCon.getRightX()); 

  private final ScoringComand operateScoring = new ScoringComand(scoreSub, ()-> xbox.getAButton(), ()-> xbox.getXButton(),
      ()-> xbox.getLeftTriggerAxis(), ()-> xbox.getRightTriggerAxis(), //controls climber
      ()-> xbox.getLeftBumperButton(), ()-> xbox.getRightBumperButton()); //controls arm
      
  //private final Intake psIntake = new Intake(intakeSub, ()-> psCon.getCrossButton());

  // Joystick Controls
  //private final DriveJoystickSwerve driveJoystick = new DriveJoystickSwerve(drivetrain, () -> stick.getY(), () -> stick.getX(), () -> stick.getTwist(),
   //() -> stick.getRawButton(7), () -> stick.getRawButton(8), () -> stick.getThrottle());

  //private final PIDButtons buttons = new PIDButtons(armSub, yeetSub, ()-> stick.getRawButton(8), ()-> stick.getRawButton(7));

  // Autos
  private final SimpleAuto simpleAuto = new SimpleAuto(scoreSub, drivetrain);
  /// SHUFFLEBOARD METHODS ///
  /**
   * Use this command to define {@link Shuffleboard} buttons using a
   * {@link ShuffleboardTab} and its add() function. You can put already defined
   * Commands,
   */
  private void configureShuffleboardData() {
    Shuffleboard.selectTab(m_tab.getTitle());

    ShuffleboardLayout drivingStyleLayout = m_tab.getLayout("driving styles", BuiltInLayouts.kList)
    .withPosition(0, 0).withSize(2, 2)
    .withProperties(Map.of("label position", "BOTTOM"));

    //Xbox controller commands
    //drivingStyleLayout.add("Xbox Drive",
    //new InstantCommand(() -> drivetrain.setDefaultCommand(drivetrainXbox), drivetrain));

    drivingStyleLayout.add("Xbox Scoring Mech",
    new InstantCommand(() -> scoreSub.setDefaultCommand(operateScoring), scoreSub));

    //PS5 controller commands
    //drivingStyleLayout.add("PS5 Drive",
    //new InstantCommand(()->  drivetrain.setDefaultCommand(drivePS), drivetrain));
    //drivingStyleLayout.add("PS5 Intake",
    //new InstantCommand(() -> intakeSub.setDefaultCommand(psIntake), intakeSub));

    //drivingStyleLayout.add("Joystick Drive",
    //new InstantCommand(() -> drivetrain.setDefaultCommand(driveJoystick), drivetrain));
 
    ShuffleboardLayout gyroSensor = m_tab.getLayout("Pidgeon", BuiltInLayouts.kGrid)
    .withPosition(2, 0).withSize(1, 3)
    .withProperties(Map.of("label position", "BOTTOM"));

    gyroSensor.addNumber("Gyro", ()-> drivetrain.getHeading().getDegrees());

    gyroSensor.addNumber("Robot Speed", ()-> drivetrain.getRobotSpeed());
    gyroSensor.addNumber("Robot Xvelo", ()-> drivetrain.getXVelo());
    gyroSensor.addNumber("Robot Yvelo", ()-> drivetrain.getYVelo());
    //gyroSensor.add("Reset",
    //new InstantCommand(()-> drivetrain.zeroHeading()));

    ShuffleboardLayout controllerLayout = m_tab.getLayout("Xbox Vals", BuiltInLayouts.kGrid)
    .withPosition(4, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    controllerLayout.addNumber("left y", () -> -xbox.getLeftY())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left x", () -> xbox.getLeftX())
    .withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("left trigger", () -> xbox.getLeftTriggerAxis())
    .withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right y", () -> -xbox.getRightY())
    .withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right x", () -> xbox.getRightX())
    .withPosition(2, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    controllerLayout.addNumber("right trigger", () -> xbox.getRightTriggerAxis())
    .withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);


    /*
    ShuffleboardLayout psLayout = m_tab.getLayout("PS5 Vals", BuiltInLayouts.kGrid)
    .withPosition(6, 0).withSize(2, 6)
    .withProperties(Map.of("label position", "BOTTOM"));
    psLayout.addNumber("left y", () -> -psCon.getLeftY())
    .withPosition(0, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    psLayout.addNumber("left x", () -> psCon.getLeftX())
    .withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    psLayout.addNumber("left trigger", () -> psCon.getL2Axis())
    .withPosition(0, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    psLayout.addNumber("right y", () -> -psCon.getRightY())
    .withPosition(2, 0).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    psLayout.addNumber("right x", () -> psCon.getRightX())
    .withPosition(2, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    psLayout.addNumber("right trigger", () -> psCon.getR2Axis())
    .withPosition(2, 2).withSize(2, 1).withWidget(BuiltInWidgets.kNumberBar);
    */
  }

  private void configureSmartDashboard(){
    //match Auto
    //m_chooser.setDefaultOption("Test Auto", new PathPlannerAuto("Test Auto"));
    //m_chooser.addOption("Test Auto", new PathPlannerAuto("Test Auto"));
    //m_chooser.addOption("ZZ Auto", new PathPlannerAuto("ZZ Auto"));
    //m_chooser.addOption("Curve Auto", new PathPlannerAuto("Curve Auto"));
    m_chooser.setDefaultOption("Simple Auto", simpleAuto);
  }

  /**   
   * Use this method to define the default commands of subsystems. 
   * Default commands are ran whenever no other commands are using a specific subsystem.
   */
  private void configureInitialDefaultCommands() {
    scoreSub.setDefaultCommand(operateScoring);
    drivetrain.setDefaultCommand(drivetrainXbox);
    //intakeSub.setDefaultCommand(psIntake);
  }
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    //Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOriented(driveAngularVelocity);
    //Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOriented(driveDirectAngle);

    //drivetrain.setDefaultCommand(driveFieldOrientedDirectAngle);
    //intake buttons
    /* 
    commandController.a().whileTrue
    (Commands.runOnce(
      ()-> {
          intakeSub.intakeOn();
      }, intakeSub));

      commandController.b().whileTrue
    (Commands.runOnce(
      ()-> {
          intakeSub.intakeOff();
      }, intakeSub));
      */

      commandController.start().onTrue(
        Commands.runOnce(
          ()->{ 
            drivetrain.zeroGyro();
          }, drivetrain));
          
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard

    // Load the path you want to follow using its name in the GUI
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //return AutoBuilder.followPath(path);
    return m_chooser.getSelected();
  }

  public void displayValues() {
  SmartDashboard.putData(drivetrain);
  SmartDashboard.putData(m_chooser);
  }
}