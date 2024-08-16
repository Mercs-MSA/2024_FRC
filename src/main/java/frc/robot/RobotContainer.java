package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ScoringMode;
import frc.robot.commands.intakeSubcommands.*;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.subsystems.SAT;
// import frc.robot.commands.CommandShooterStart;
import frc.robot.commands.CommandShooterStop;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // public final CommandXboxController driver = new CommandXboxController(0);
    // public final CommandXboxController operator = new CommandXboxController(1);
    public final XboxController driver = new XboxController(0);
    //  public final PS5Controller driver = new PS5Controller(0);


    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Intake m_Intake = new Intake();
    public static final SAT m_SAT = new SAT();
    // public ApriltagVision m_ApriltagVision = new ApriltagVision();

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            put("marker1", Commands.print("Finished 1 Piece"));
            put("marker2", Commands.print("Finished 3-4 Piece"));
            
      
            //put("Test Scoring Mode Amp", new CommandChangeScoringMode(ScoringMode.AMP));

            // Intake handoff commands
   
            // shooting commmands
   

        }  
    };

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getLeftY(), 
                () -> -driver.getLeftX(), 
                () -> -driver.getRightX(), 
                () -> false // just hardcoded field centric... could make this a button if we want
            )
        );

        NamedCommands.registerCommands(autonomousCommands);

        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("DONOTHING"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
        

        
    }

    public void configureButtonBindings() {
        driverControls();
        operatorControls();
        //manualTesting();
    }

    public void driverControls(){
        //driver.start().and(driver.back()).onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));

       
      
        
        
        // driver.leftBumper().onTrue(new CommandBaseStartPosition(m_SAT));
        // driver.leftBumper().onTrue(new InstantCommand(() -> m_intake.reverseIntakeMotor()));

        //  driver.rightBumper()
        //  .onTrue(
        //      new SequentialCommandGroup(
        //          new CommandPivotHandoffPosition(m_SAT),
        //          new CommandIndexStart(m_intake),
        //          //new WaitCommand(0.5),
        //          new CommandIntakeStart(m_intake),
        //          new CommandIntakeWaitForNote(m_intake),
        //          // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
        //          new WaitCommand(0.3), // This just worked more reliably and more easily than the sensor did
        //          new CommandIntakeStop(m_intake),
        //          new InstantCommand(() -> m_SAT.shootNote(35)), 
        //          new WaitCommand(3),
        //          new CommandIndexStop(m_intake),
        //          new InstantCommand(() -> m_SAT.stopShooter())
        //      )
        //  );

        // should be fixed now, so dodn't need this

        // driver.leftBumper()
        // .onTrue(
        //     new InstantCommand(() -> m_intake.stopIntakeMotor())
        // );

        // driver.a().onTrue(new SequentialCommandGroup(
        //     new PrintCommand("this started"),
        //     s_Swerve.driveToPose(
        //         () -> s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getX(),
        //         () -> s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getY(),
        //         () -> Constants.Vision.getRobotHeading(m_GamePieceVision.getGamePieceYaw())
        //     ),
        //     new PrintCommand("this finished")
        //     ));

        // driver.a().onTrue( //podium 
        //     new CommandSwerveToPoseProxy(
        //         s_Swerve,
        //         () -> Constants.Vision.getPose("podium").getX(),
        //         () -> Constants.Vision.getPose("podium").getY(),
        //         () -> Constants.Vision.getPose("podium").getRotation().getDegrees())
        //     );

        // driver.b().onTrue( //speaker center 
        //     new CommandSwerveToPoseProxy(
        //         s_Swerve,
        //         () -> Constants.Vision.getPose("sub").getX(),
        //         () -> Constants.Vision.getPose("sub").getY(),
        //         () -> Constants.Vision.getPose("sub").getRotation().getDegrees())
        //     );

        

        // driver.rightBumper().onTrue( //speaker center 
        //     new CommandSwerveToPoseProxy(
        //         s_Swerve,
        //         () -> Constants.Vision.getPose("subright").getX(),
        //         () -> Constants.Vision.getPose("subright").getY(),
        //         () -> Constants.Vision.getPose("subright").getRotation().getDegrees())
        //     );

        // driver.y().onTrue(
        //     new CommandSwerveToPoseProxy(
        //         s_Swerve,
        //         () -> s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getX(),
        //         () -> s_Swerve.poseEstimator.getEstimatedPosition().getTranslation().getY(),
        //         () -> m_GamePieceVision.calculateGamePieceHeading()
        //     )
        // );


        // driver.a()
        // .whileTrue(
        //         new CommandSwerveTurnToNote(s_Swerve, m_GamePieceVision))
        // .onFalse(
        //     new SequentialCommandGroup(
        //         new InstantCommand(() -> s_Swerve.drive(new Translation2d(), 0, false, false)),
        //         new CommandIntakeStop(m_intake),
        //         new CommandIndexStop(m_intake),
        //         new CommandPivotStartPosition(m_SAT)));
            
        // driver.b().onTrue(new CommandSwerveDriveToNote(s_Swerve, m_GamePieceVision));


    //      driver.rightBumper()
    //         .onTrue(
    //             new SequentialCommandGroup(
    //             )
    //         ); 
    // }
    }
    public void operatorControls(){
       // operator.pov(0).onTrue(new CommandChangeScoringMode(ScoringMode.WING));
     

        

            

     

        // operator.start().onTrue(                               
        //     Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
        // );
        //     operator.b().onTrue(
        //         new SequentialCommandGroup(
        //             new CommandIntakeStop(m_intake),
        //             new CommandPivotStartPosition(m_SAT),
        //             new CommandIndexStop(m_intake),
        //             new CommandShooterStop(m_SAT),

        //             new CommandShooterReverse(m_SAT),
        //             new CommandIndexMoveNoteToFiringPosition(m_intake),
        //             new WaitCommand(0.2),
        //             new CommandIndexStop(m_intake),
        //             new CommandShooterStop(m_SAT)
        // )
        //     );

    //     operator.y().onTrue(
    //     new CommandShooterStart(m_SAT)
    //    );

    //    operator.b().onTrue(
    //     new CommandShooterStop(m_SAT)
    //    );

    //      operator.x().onTrue(
    //     new CommandIntakeReverse(m_Intake)
    //    );

       

        // operator.b()
        //     .whileTrue(new InstantCommand(() -> m_intake.reverseIntakeMotor()))
        //     .onFalse(
        //             new SequentialCommandGroup(
        //                 new CommandIntakeStop(m_intake),
        //                 new CommandIndexStop(m_intake),        
        //                 new CommandPivotStartPosition(m_SAT)      
        //             )
        //     );
        // when you press the x button:
        // operator.x()
        //    .onTrue(
        //     new ConditionalCommand(
        //         new ConditionalCommand(
        //             // if the intake system is on and you have a note, the system does nothing
        //             new InstantCommand(),

        //             // if the intake system is on and you don't have a note, the system turns off
        //             new SequentialCommandGroup(
        //                 new CommandIntakeStop(m_intake),
        //                 new CommandIndexStop(m_intake),        
        //                 new CommandPivotStartPosition(m_SAT)      
        //             ),

        //             // this checks if we have a note
        //             () -> IntakeConstants.kRobotHasNote == true
        //         ),
        //         new ConditionalCommand(
        //             // if the intake system is off and you have a note, the system does nothing
        //             new InstantCommand(),

        //             // if the intake system is off and you don't have a note, the system turns on
        //             new SequentialCommandGroup(
        //                 new CommandPivotHandoffPosition(m_SAT),
        //                 new CommandIntakeStart(m_intake),
        //                 new CommandIndexStart(m_intake),
        //                 new CommandIntakeWaitForNote(m_intake),
        //                 new CommandChangeRobotHasNote(true),
        //                 // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
        //                 new WaitCommand(0.3), // This just worked more reliably and more easily than the sensor did
        //                 new CommandIntakeStop(m_intake),
        //                 new CommandIndexStop(m_intake),
        //                 new CommandPivotStartPosition(m_SAT)
        //             ),

        //             // this checks if we have a note
        //             () -> IntakeConstants.kRobotHasNote == true
        //         ),
        //         // this checks if the intake system is on
        //         () -> m_intake.getIndexMotorSpeed() != 0
        //     ));
        

        // operator.y()
        //     .whileTrue(
        //         m_climber.climbDownRightCommand()
        //     );
    
        // operator.x()
        //     .whileTrue(
        //         m_climber.climbUpRightCommand()
        //     );
            
        // operator.a()
        //     .whileTrue(
        //         m_climber.climbUpLeftCommand()
        //     );
    
        // operator.b()
        //     .whileTrue(
        //         m_climber.climbDownLeftCommand()
        //     );
    }

    public void manualTesting(){
      


        // driver.pov(0).onTrue(new CommandIndexStart(m_intake));
        // driver.pov(180).onTrue(new CommandIndexStop(m_intake));

        // driver.pov(90).onTrue(new CommandIntakeStart(m_intake));
        // driver.pov(270).onTrue(new CommandIntakeStop(m_intake));

        // driver.leftBumper().onTrue(new CommandShooterStart(m_SAT));
        // driver.rightBumper().onTrue(new CommandShooterStop(m_SAT));

        // operator.b().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesLeft(operator.getLeftY())))
        // .whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesRight(operator.getRightY())));

        // operator.a().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSides(operator.getLeftY())));


        //Testing for amp with override DO NOT DELETE 

        // operator.y()
        // .onTrue(new SequentialCommandGroup(
        //     new CommandBaseScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_1.motor1_base),
        //     new WaitCommand(1.0),
        //     new PrintCommand("base stage 1"),
        //     new CommandPivotScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_1.pivot),
        //     new WaitCommand(1.0),
        //     new PrintCommand("pivot stage 1"),
        //     new CommandBaseScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_2.motor1_base),
        //     new WaitCommand(1.0),
        //     new PrintCommand("base stage 2"),
        //     new CommandPivotScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_2.pivot),
        //     new WaitCommand(1.0),
        //     new PrintCommand("pivot stage 2"),
        //     new CommandBaseScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_3.motor1_base),
        //     new WaitCommand(1.0),
        //     new PrintCommand("base stage 3"),
        //     new CommandPivotScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_3.pivot),
        //     new PrintCommand("pivot stage 3")



        // ));


        // operator.a()
        // .onTrue(new SequentialCommandGroup(
        //     new CommandBaseScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_2.motor1_base),
        //     new WaitCommand(1.0),
        //     new CommandPivotScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_2.pivot),
        //     new WaitCommand(1.0),
        //     new CommandBaseScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_1.motor1_base),
        //     new WaitCommand(1.0),
        //     new CommandPivotScoringPosition(m_SAT, true, Constants.SATConstants.AMP_STAGE_1.pivot),
        //     new WaitCommand(1.0),
        //     new CommandBaseScoringPosition(m_SAT, true, Constants.SATConstants.START.motor1_base),
        //     new WaitCommand(1.0),
        //     new CommandPivotScoringPosition(m_SAT, true, Constants.SATConstants.START.pivot)



        // ));

        // operator.x()
        // .onTrue(new SequentialCommandGroup(
        //     new CommandIndexStart(m_intake),
        //     new CommandShooterStart(m_SAT)
        // ))
        // .onFalse(new SequentialCommandGroup(
        //     new CommandIndexStop(m_intake),
        //     new CommandShooterStop(m_SAT)

        // ));


        // // operator.start()
        //     .onTrue(new CommandChangeScoringMode(ScoringMode.AMP));
        // operator.a()
        //     .onTrue(new CommandPivotScoringPosition(m_SAT));
        // operator.b()
        //     .onTrue(new CommandBaseScoringPosition(m_SAT));
        // operator.x()
        //     .onTrue(new CommandPivotStageTwoPosition(m_SAT));
        // operator.y()
        //     .onTrue(new CommandBaseStageTwoPosition(m_SAT));
        // operator.leftBumper()
        //     .onTrue(new CommandBaseStartPosition(m_SAT));
        // operator.rightBumper()
        //     .onTrue(new CommandPivotStartPosition(m_SAT));

        /* TESTING SEQUENCE
         * 1. start
         * 2. a
         * 3. b
         * 4. x
         * 5. y
         * 6. b
         * 7. a
         * 8. left bumper
         * 9. right bumper
         */ 
    }

    public void pitTestControls(){
        //swerve
        // s_Swerve.setDefaultCommand(
        //     new TeleopSwerve(
        //         s_Swerve, 
        //         () -> -tester.getLeftY(), 
        //         () -> -tester.getLeftX(), 
        //         () -> -tester.getRightX(), 
        //         () -> false // just hardcoded field centric... could make this a button if we want
        //     )
        // );

      }

    public void operatorTesting(){
        /************************/
        /*                      */
        /*   Operator Buttons   */
        /*                      */
        /************************/

        // operator.axisGreaterThan(5, 0.5)
        //     .whileTrue(
        //         m_climber.climbDownCommand()
        //     );

        // operator.axisLessThan(5, -0.5)
        //     .whileTrue(
        //         m_climber.climbUpCommand()
        //     );

        // operator.axisLessThan(5, 0.5)
        //     .whileTrue(
        //         m_climber.climbMotorStop()
        //     );

        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}