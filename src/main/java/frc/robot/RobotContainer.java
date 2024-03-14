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
import frc.robot.commands.CommandShooterStart;
import frc.robot.commands.CommandShooterStop;
import frc.robot.commands.CommandSwerveDriveToNote;
import frc.robot.commands.CommandSwerveToPoseProxy;
import frc.robot.commands.CommandSwerveTurnToNote;
import frc.robot.commands.CommandChangeRobotHasNote;
import frc.robot.commands.CommandChangeScoringMode;
import frc.robot.commands.CommandDriveToPose;
import frc.robot.commands.CommandShooterReverse;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.BaseSubcommands.*;
import frc.robot.commands.IntakeSubcommands.*;
import frc.robot.commands.IndexSubcommands.*;
import frc.robot.commands.PivotSubcommands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.base.Base;
import frc.robot.subsystems.climber.climber;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.ApriltagVision;
// import frc.robot.subsystems.vision.ApriltagVision;
import frc.robot.subsystems.vision.CustomGamePieceVision;
import frc.robot.Constants.AllianceFlipUtil;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.ScoringConstants.ScoringMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController operator = new CommandXboxController(1);
    public final CommandXboxController tester = new CommandXboxController(2);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public final Index m_index = new Index();
    public final Shooter m_shooter = new Shooter();
    public final Pivot m_pivot = new Pivot();
    public final Base m_base = new Base();
    public final Intake m_intake = new Intake();
    public final climber m_climber = new climber();
    public CustomGamePieceVision m_GamePieceVision = new CustomGamePieceVision("note_yaw", "note_dist");
    // public ApriltagVision m_ApriltagVision = new ApriltagVision();

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    // public CommandSwerveToNote commandSwerveToNote = new CommandSwerveToNote(s_Swerve, m_GamePieceVision);

    Map<String, Command> autonomousCommands = new HashMap<String,Command>() {
        {
            put("marker1", Commands.print("Finished 1 Piece"));
            put("marker2", Commands.print("Finished 3-4 Piece"));
            
            put("Test Start Intake", new CommandIntakeStart(m_intake));
            put("Test Start Index", new CommandIndexStart(m_index));
            put("Test Stop Intake", new CommandIntakeStop(m_intake));
            put("Test Stop Index", new CommandIndexStop(m_index));
            put("Test Start Shooter", new CommandShooterStart(m_shooter));
            put("Test Stop Shooter", new CommandShooterStop(m_shooter));
            put("Test Scoring Mode Wing", new CommandChangeScoringMode(ScoringMode.WING));
            put("Test Scoring Mode Subwoofer", new CommandChangeScoringMode(ScoringMode.SUBWOOFER));
            put("Test Scoring Mode Podium", new CommandChangeScoringMode(ScoringMode.PODIUM));
            put("Test Scoring Mode Amp", new CommandChangeScoringMode(ScoringMode.AMP));

            // Intake handoff commands
            put("pivot handoff position", new CommandPivotHandoffPosition(m_pivot));
            put("intake start", new CommandIntakeStart(m_intake));
            put("wait for note", new CommandIntakeWaitForNote(m_intake));
            put("wait 0.2", new WaitCommand(0.2));
            put("intake stop", new CommandIntakeStop(m_intake));

            // shooting commmands
            put("scoring mode subwoofer", new CommandChangeScoringMode(ScoringMode.SUBWOOFER));
            put("pivot scoing position", new CommandPivotScoringPosition(m_pivot));
            put("base scoring position", new CommandBaseScoringPosition(m_base));
            put("note to firing position", new CommandIndexMoveNoteToFiringPosition(m_index));
            put("shooter start", new CommandShooterStart(m_shooter));
            put("shooter start sub", new CommandShooterStart(m_shooter));
            put("index start", new CommandIndexStart(m_index));
            put("wait 0.3", new WaitCommand(0.3));
            put("index stop", new CommandIndexStop(m_index));
            put("shooter stop", new CommandShooterStop(m_shooter));
            put("base start position", new CommandBaseStartPosition(m_base));
            put("pivot start position", new CommandPivotStartPosition(m_pivot));

            put("scoring mode podium", new CommandChangeScoringMode(ScoringMode.PODIUM));
            put("score sub note", new SequentialCommandGroup(
                    new CommandChangeScoringMode(ScoringMode.SUBWOOFER), // pivot move to whatever current mode is
                    new CommandIntakeStart(m_intake),
                    new CommandBaseScoringPosition(m_base),
                    // new CommandIndexMoveNoteToFiringPosition(m_intake),
                   new CommandIndexReverse(m_index),
                  //  new WaitCommand(0.05),
                    // new CommandIndexStopInstant(m_intake),
                    new CommandShooterStart(m_shooter), // shoot with speed of whatever current mode is
                    new WaitCommand(0.25),
                    new CommandIndexStart(m_index),
                    new WaitCommand(0.3), // waiting for the note to leave robot
                    new ParallelCommandGroup( // Since Index and Shooter are different subsystems, stop both at same time
                        new CommandIndexStop(m_index),
                        new CommandShooterStop(m_shooter)
                    ),
                    new CommandIntakeStop(m_intake),
                    new CommandChangeRobotHasNote(false),
                    new CommandBaseStartPosition(m_base),
                    new CommandPivotStartPosition(m_pivot)
                ));
                put("score podium note", new SequentialCommandGroup(
                    new CommandChangeScoringMode(ScoringMode.PODIUM),
                    new CommandPivotScoringPosition(m_pivot), // pivot move to whatever current mode is
                    new CommandBaseScoringPosition(m_base), // base move to whatever current mode is
                    new ConditionalCommand( // IF WE NEED TO SCORE AMP...
                        new CommandPivotStageTwoPosition(m_pivot), // A 2nd pivot rotation is needed
                        new InstantCommand(), // if we're not scoring amp, do nothing
                        () -> ScoringConstants.currentScoringMode == ScoringMode.AMP
                    ),
                    new CommandIndexMoveNoteToFiringPosition(m_index),
                    new WaitCommand(0.1),
                    new CommandShooterStart(m_shooter), // shoot with speed of whatever current mode is
                    new CommandIndexStart(m_index),
                    new WaitCommand(0.3), // waiting for the note to leave robot
                    new ParallelCommandGroup( // Since Index and Shooter are different subsystems, stop both at same time
                        new CommandIndexStop(m_index),
                        new CommandShooterStop(m_shooter)
                    ),
                    new CommandChangeRobotHasNote(false),
                    new ConditionalCommand( // IF WE JUST SCORED AMP...
                        new CommandPivotScoringPosition(m_pivot),  // A 2nd pivot rotation is needed
                        new InstantCommand(), // if we're not scoring amp, do nothing
                        () -> ScoringConstants.currentScoringMode == ScoringMode.AMP
                    ),
                    new CommandBaseStartPosition(m_base),
                    new CommandPivotStartPosition(m_pivot)
                ));
            put("score wing note", new SequentialCommandGroup(
                    new CommandChangeScoringMode(ScoringMode.WING),
                    new CommandPivotScoringPosition(m_pivot), // pivot move to whatever current mode is
                    new CommandBaseScoringPosition(m_base), // base move to whatever current mode is
                    new ConditionalCommand( // IF WE NEED TO SCORE AMP...
                        new CommandPivotStageTwoPosition(m_pivot), // A 2nd pivot rotation is needed
                        new InstantCommand(), // if we're not scoring amp, do nothing
                        () -> ScoringConstants.currentScoringMode == ScoringMode.AMP
                    ),
                    new CommandIndexMoveNoteToFiringPosition(m_index),
                    // new WaitCommand(0.1),
                    new CommandShooterStart(m_shooter), // shoot with speed of whatever current mode is
                    new CommandIndexStart(m_index),
                    new WaitCommand(0.3), // waiting for the note to leave robot
                    new ParallelCommandGroup( // Since Index and Shooter are different subsystems, stop both at same time
                        new CommandIndexStop(m_index),
                        new CommandShooterStop(m_shooter)
                    ),
                    new CommandChangeRobotHasNote(false),
                    new ConditionalCommand( // IF WE JUST SCORED AMP...
                        new CommandPivotScoringPosition(m_pivot),  // A 2nd pivot rotation is needed
                        new InstantCommand(), // if we're not scoring amp, do nothing
                        () -> ScoringConstants.currentScoringMode == ScoringMode.AMP
                    ),
                    new CommandBaseStartPosition(m_base),
                    new CommandPivotStartPosition(m_pivot)
                ));
            put("Intake Note", new SequentialCommandGroup(
                                new CommandBaseStartPosition(m_base),
                                new CommandIndexStart(m_index),
                                new CommandPivotHandoffPosition(m_pivot),
                                new CommandIntakeStart(m_intake),
                                new CommandShooterReverse(m_shooter),
                                // new CommandIndexWaitForNote(m_intake),
                                new WaitCommand(1),
                                new CommandChangeRobotHasNote(true),
                                // new WaitCommand(0.25),
                                // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
                                // new WaitCommand(0.75),
                                // new CommandIndexStop(m_intake),
                                
                                // new CommandIndexMoveNoteToFiringPosition(m_intake),
                                new CommandIndexStop(m_index),
                                new CommandIntakeStop(m_intake),
                                new CommandPivotStartPosition(m_pivot),
                                new CommandShooterStop(m_shooter)
                            ));
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
        // manualTesting();
    }

    public void driverControls(){
        driver.start().and(driver.back()).onTrue(Commands.runOnce(() -> s_Swerve.zeroHeading(), s_Swerve));



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

        driver.leftBumper().onTrue(new InstantCommand(() -> m_intake.reverseIntakeMotor())).onFalse(new InstantCommand(() -> m_intake.stopIntakeMotor()));

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


         driver.rightBumper()
            .onTrue(
                new SequentialCommandGroup(
                    new CommandPivotScoringPosition(m_pivot), // pivot move to whatever current mode is
                    new CommandBaseScoringPosition(m_base), // base move to whatever current mode is
                    // new ConditionalCommand( // IF WE NEED TO SCORE AMP...
                    //     new SequentialCommandGroup(
                    //         new CommandPivotStageTwoPosition(m_SAT), // A 2nd pivot rotation is needed
                    //         new CommandBaseStageTwoPosition(m_SAT)
                    //     ),
                    //     new InstantCommand(), // if we're not scoring amp, do nothing
                    //     () -> ScoringConstants.currentScoringMode == ScoringMode.AMP
                    // ),
                    new CommandIndexReverse(m_index),
                    new WaitCommand(0.20),
                      new ParallelCommandGroup( 
                        new CommandShooterStart(m_shooter),
                        new CommandIndexStop(m_index)
                    ), // shoot with speed of whatever current mode is)
                    // new CommandIndexMoveNoteToFiringPosition(m_intake),
                   
                    // new WaitCommand(1), // waiting for the note to leave robot
                    new CommandIndexStart(m_index),
                    new WaitCommand(1), // waiting for the note to leave robot
                    new ParallelCommandGroup( // Since Index and Shooter are different subsystems, stop both at same time
                        new CommandIndexStop(m_index),
                        new CommandShooterStop(m_shooter)
                    ),
                    new CommandChangeRobotHasNote(false),
                    // new ConditionalCommand( // IF WE JUST SCORED AMP...
                    //     new SequentialCommandGroup(
                    //         new CommandBaseScoringPosition(m_SAT),
                    //         new CommandPivotScoringPosition(m_SAT)  // A 2nd pivot rotation is needed
                    //     ),
                    //     new InstantCommand(), // if we're not scoring amp, do nothing
                    //     () -> ScoringConstants.currentScoringMode == ScoringMode.AMP
                    // ),
                    new WaitCommand(1),
                    new CommandBaseStartPosition(m_base),
                    new CommandPivotStartPosition(m_pivot)
                )
            ); 
    }

    public void operatorControls(){
        operator.pov(0).onTrue(new CommandChangeScoringMode(ScoringMode.WING));
        operator.pov(90).onTrue(new CommandChangeScoringMode(ScoringMode.SUBWOOFER));
        operator.pov(180).onTrue(new CommandChangeScoringMode(ScoringMode.PODIUM));
        // operator.leftBumper().onTrue(new CommandIndexStart(m_intake)));

        // operator.pov(270).onTrue(new CommandChangeScoringMode(ScoringMode.AMP));

        

        operator.x()
                .onTrue(
                    // new ConditionalCommand(
                    //     new ConditionalCommand(
                    //         // if the intake system is on and you have a note, the system does nothing
                    //         new InstantCommand(),

                    //         // if the intake system is on and you d9on't have a note, the system turns off
                    //         new SequentialCommandGroup(
                    //             new CommandIntakeStop(m_intake),
                    //             new CommandIndexStop(m_intake),        
                    //             new CommandPivotStartPosition(m_SAT)      
                    //         ),

                    //         // this checks if we have a note
                    //         () -> IntakeConstants.kRobotHasNote == true
                    //     ),
                    //     new ConditionalCommand(
                    //         // if the intake system is off and you have a note, the system does nothing
                    //         new InstantCommand(),

                            // if the intake system is off and you don't have a note, the system turns on
                            new SequentialCommandGroup(
                                new CommandBaseStartPosition(m_base),
                                new CommandIndexStart(m_index),
                                new CommandPivotHandoffPosition(m_pivot),
                                new CommandIntakeStart(m_intake),
                                new CommandShooterReverse(m_shooter),
                                // new CommandIndexWaitForNote(m_intake),
                                // new WaitCommand(2),
                                // new CommandChangeRobotHasNote(true),
                                // new WaitCommand(0.25),
                                // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
                                // new WaitCommand(0.75),
                                // new CommandIndexStop(m_intake),
                                
                                // new CommandIndexMoveNoteToFiringPosition(m_intake),
                                // new CommandIndexStop(m_intake),
                                // new CommandIntakeStop(m_intake),
                                // new CommandPivotStartPosition(m_SAT),
                                new CommandShooterStop(m_shooter)
                            )

                    //         // this checks if we have a note
                    //         () -> IntakeConstants.kRobotHasNote == true
                    //     ),
                    //     // this checks if the intake system is on
                    //     () -> m_intake.getIndexMotorSpeed() != 0
                    // )
                );

        operator.x().onFalse(
            new SequentialCommandGroup(
                //new CommandIndexMoveNoteToFiringPosition(m_intake),
                new CommandIndexStop(m_index),
                new CommandIntakeStop(m_intake)
        ));

        operator.start().onTrue(
            Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())
        );
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

        operator.a().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesLeft(operator.getLeftY())))
        .whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesRight(operator.getRightY())));
        

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
        operator.pov(0).whileTrue(new RunCommand(() -> m_pivot.pivotGoToPositionIncrement(0.25), m_pivot));
        operator.pov(180).whileTrue(new RunCommand(() -> m_pivot.pivotGoToPositionIncrement(-0.25), m_pivot));
        operator.pov(90).whileTrue(new RunCommand(() -> m_base.baseGoToPositionIncrement(0.5), m_pivot));
        operator.pov(270).whileTrue(new RunCommand(() -> m_base.baseGoToPositionIncrement(-0.5), m_pivot));


        driver.pov(0).onTrue(new CommandIndexStart(m_index));
        driver.pov(180).onTrue(new CommandIndexStop(m_index));

        driver.pov(90).onTrue(new CommandIntakeStart(m_intake));
        driver.pov(270).onTrue(new CommandIntakeStop(m_intake));

        driver.leftBumper().onTrue(new CommandShooterStart(m_shooter));
        driver.rightBumper().onTrue(new CommandShooterStop(m_shooter));

        operator.b().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesLeft(operator.getLeftY())))
        .whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesRight(operator.getRightY())));

        operator.a().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSides(operator.getLeftY())));

                operator.x()
                .onTrue(
                    // new ConditionalCommand(
                    //     new ConditionalCommand(
                    //         // if the intake system is on and you have a note, the system does nothing
                    //         new InstantCommand(),

                    //         // if the intake system is on and you d9on't have a note, the system turns off
                    //         new SequentialCommandGroup(
                    //             new CommandIntakeStop(m_intake),
                    //             new CommandIndexStop(m_intake),        
                    //             new CommandPivotStartPosition(m_SAT)      
                    //         ),

                    //         // this checks if we have a note
                    //         () -> IntakeConstants.kRobotHasNote == true
                    //     ),
                    //     new ConditionalCommand(
                    //         // if the intake system is off and you have a note, the system does nothing
                    //         new InstantCommand(),

                            // if the intake system is off and you don't have a note, the system turns on
                            new SequentialCommandGroup(
                                new CommandBaseStartPosition(m_base),
                                new CommandIndexStart(m_index),
                                new CommandPivotHandoffPosition(m_pivot),
                                new CommandIntakeStart(m_intake),
                                new CommandShooterReverse(m_shooter),
                                // new CommandIndexWaitForNote(m_intake),
                                // new WaitCommand(2),
                                // new CommandChangeRobotHasNote(true),
                                // new WaitCommand(0.25),
                                // Once we see a note on the bottom sensors, then the wait command below is for the handoff to complete
                                // new WaitCommand(0.75),
                                // new CommandIndexStop(m_intake),
                                
                                // new CommandIndexMoveNoteToFiringPosition(m_intake),
                                // new CommandIndexStop(m_intake),
                                // new CommandIntakeStop(m_intake),
                                // new CommandPivotStartPosition(m_SAT),
                                new CommandShooterStop(m_shooter)
                            )

                    //         // this checks if we have a note
                    //         () -> IntakeConstants.kRobotHasNote == true
                    //     ),
                    //     // this checks if the intake system is on
                    //     () -> m_intake.getIndexMotorSpeed() != 0
                    // )
                );

        // operator.start()
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
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -tester.getLeftY(), 
                () -> -tester.getLeftX(), 
                () -> -tester.getRightX(), 
                () -> false // just hardcoded field centric... could make this a button if we want
            )
        );

        //outake
        tester.a().and(tester.pov(0).whileTrue(new RunCommand(() -> m_pivot.pivotGoToPositionIncrement(0.25), m_pivot)));
        tester.a().and(tester.pov(180).whileTrue(new RunCommand(() -> m_pivot.pivotGoToPositionIncrement(-0.25), m_pivot)));
        tester.a().and(tester.pov(90).whileTrue(new RunCommand(() -> m_base.baseGoToPositionIncrement(0.5), m_pivot)));
        tester.a().and(tester.pov(270).whileTrue(new RunCommand(() -> m_base.baseGoToPositionIncrement(-0.5), m_pivot)));
        
        tester.leftBumper().onTrue(new CommandShooterStart(m_shooter));
        tester.rightBumper().onTrue(new CommandShooterStop(m_shooter));

        //intake
        tester.b().and(tester.pov(0).onTrue(new CommandIndexStart(m_index)));
        tester.b().and(tester.pov(180).onTrue(new CommandIndexStop(m_index)));

        tester.b().and(tester.pov(90).onTrue(new CommandIntakeStart(m_intake)));
        tester.b().and(tester.pov(270).onTrue(new CommandIntakeStop(m_intake)));

        //climber
        tester.x().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesLeft(operator.getLeftY())))
        .whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSidesRight(operator.getRightY())));

        tester.y().whileTrue(new RunCommand(() -> m_climber.incrementalClimbBothSides(operator.getLeftY())));
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

        operator.a().onTrue(m_climber.climbDownCommand());
        operator.y().onTrue(m_climber.climbUpCommand());
        operator.x().onTrue(m_climber.climbMidLeftCommand());
        operator.b().onTrue(m_climber.climbMidRightCommand());
    }

    // /**
    //  * Use this to pass the autonomous command to the main {@link Robot} class.
    //  *
    //  * @return the command to run in autonomous
    //  */
    // public Command getAutonomousCommand() {
    //     return autoChooser.getSelected();
    // }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(AllianceFlipUtil.apply(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0))))),

            shootFirstNoteSub(),
            intakeNote(),
            
            new ParallelCommandGroup(
                new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(3.1, 5.50, Rotation2d.fromDegrees(0)))), 
                new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
                intakeBasePivot()
            ),  

            stopIntakeNote(),
            
            new ParallelCommandGroup(
                new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(1.48, 5.54, Rotation2d.fromDegrees(0)))), //sub
                new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
                moveStuffSub()
            ),

            shootNoteSub(),
            intakeNote(),

            new ParallelCommandGroup(
                new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(2.67 + 0.3, 4.09 - 0.35, Rotation2d.fromDegrees(-30)))), 
                new CommandChangeScoringMode(ScoringMode.PODIUM),
                intakeBasePivot()
            ),  
            
            stopIntakeNote(),
            moveStuffPodium(),
            shootNotePodium(),
            new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0)))), //sub

            
            // new ParallelCommandGroup(
            //     new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0)))), //sub
            //     new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            //     moveStuffSub()
            // ),

            // shootNoteSub(),
            // intakeNote(),

            new ParallelCommandGroup(
                new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(2.77 + 0.35, 7.09 + 0.15, Rotation2d.fromDegrees(30)))), 
                new CommandChangeScoringMode(ScoringMode.PODIUM),
                intakeBasePivot()
            ),  

            stopIntakeNote(),
            moveStuffPodium(),
            shootNotePodium());

            // new ParallelCommandGroup(
            //     new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0)))), //sub
            //     new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            //     moveStuffSub()
            // ),

            // intakeNote(),

            // new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(2.67 + 0.3, 4.09 - 0.3, Rotation2d.fromDegrees(-30)))),

            // stopIntakeNote(),

            // new ParallelCommandGroup(
            //     new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(1.38, 5.54, Rotation2d.fromDegrees(0)))), //sub
            //     new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            //     moveStuffSub()
            // ),
            // shootNoteSub(),
            // intakeNote(),

            // new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(2.77 + 0.35, 7.09 - 0.35, Rotation2d.fromDegrees(30)))),

            // new ParallelCommandGroup(
            //     new CommandDriveToPose(s_Swerve, AllianceFlipUtil.apply(new Pose2d(2.25, 6.14, Rotation2d.fromDegrees(30)))),
            //     moveStuffPodium()
            // ),

            // stopIntakeNote(),

            // shootNotePodium(),
            // new CommandIntakeStop(m_intake)

    }

    public Command intakeNote(){
        Constants.State.currentRobotState = Constants.State.robotState.INTAKE;
        return new SequentialCommandGroup(
            // new CommandBaseStartPosition(m_SAT),
            // new CommandPivotStartPosition(m_SAT),
            // new CommandShooterStop(m_SAT),
            new CommandIntakeStart(m_intake),
            new CommandIndexStart(m_index),
            new CommandShooterReverse(m_shooter)
            
            // new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            // new CommandBaseScoringPosition(m_SAT), 
            // new CommandPivotScoringPosition(m_SAT)
            
            // new CommandPivotHandoffPosition(m_SAT),
            
        );
    }

    public Command intakeBasePivot(){
        return new SequentialCommandGroup(
            new CommandBaseStartPosition(m_base),
            // new CommandPivotStartPosition(m_SAT)            
            new CommandPivotHandoffPosition(m_pivot)
            
        );
    }

    public Command stopIntakeNote(){
        Constants.State.currentRobotState = Constants.State.robotState.IDLE;
        return new SequentialCommandGroup(
            new WaitCommand(0.5),
            // new CommandIntakeStop(m_intake),
            // new CommandShooterStart(m_SAT),
            new CommandIndexStop(m_index),
            new CommandChangeScoringMode(ScoringMode.SUBWOOFER)
        );
    }

    public Command moveStuffSub(){
        return new SequentialCommandGroup(
        new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
        new CommandIndexReverse(m_index),
        new WaitCommand(0.2),
        new CommandIndexStop(m_index),
        new CommandShooterStart(m_shooter),
        new CommandBaseScoringPosition(m_base), 
        new CommandPivotScoringPosition(m_pivot));
    }

    public Command moveStuffPodium(){
        return new SequentialCommandGroup(
        new CommandChangeScoringMode(ScoringMode.PODIUM),
        new CommandIndexReverse(m_index),
        new WaitCommand(0.15),
        new CommandIndexStop(m_index),
        new CommandShooterStart(m_shooter),
        new CommandBaseScoringPosition(m_base), 
        new CommandPivotScoringPosition(m_pivot));
    }

    public Command shootFirstNoteSub(){
        Constants.State.currentRobotState = Constants.State.robotState.SCORING;
        return new SequentialCommandGroup(
            new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            new CommandBaseScoringPosition(m_base), 
            new CommandPivotScoringPosition(m_pivot),
            new CommandIndexReverse(m_index),
            new WaitCommand(0.175),
              new ParallelCommandGroup( 
                new CommandShooterStart(m_shooter),
                new CommandIndexStop(m_index)
            ),
            new CommandIndexStart(m_index),
            new WaitCommand(0.35),
            new ParallelCommandGroup( 
                new CommandIndexStop(m_index),
                new CommandShooterStop(m_shooter)
            ),
            new CommandChangeRobotHasNote(false)
            // new WaitCommand(0.25),
            // new CommandBaseStartPosition(m_SAT),
            // new CommandPivotStartPosition(m_SAT)
        );
    }

    public Command shootNoteSub(){
        Constants.State.currentRobotState = Constants.State.robotState.SCORING;
        return new SequentialCommandGroup(
            new CommandChangeScoringMode(ScoringMode.SUBWOOFER),
            new CommandBaseScoringPosition(m_base), 
            new CommandPivotScoringPosition(m_pivot),
            new CommandIndexStart(m_index),
            new CommandShooterStart(m_shooter),
            // new CommandIndexReverse(m_intake),
            // new WaitCommand(0.20),
            //   new ParallelCommandGroup( 
            //     new CommandShooterStart(m_SAT),
            //     new CommandIndexStop(m_intake)
            // ),
            // new CommandShooterStart(m_SAT),
            // new CommandIndexStart(m_intake),
            new WaitCommand(0.15),
            new ParallelCommandGroup( 
                new CommandIndexStop(m_index),
                new CommandShooterStop(m_shooter)
            ),
            // new CommandShooterStop(m_SAT),
            new CommandChangeRobotHasNote(false)
            // new WaitCommand(0.15),
            // new CommandBaseStartPosition(m_SAT),
            // new CommandPivotStartPosition(m_SAT)
        );
    }

    public Command shootNotePodium(){
        Constants.State.currentRobotState = Constants.State.robotState.SCORING;
        return new SequentialCommandGroup(
            new CommandChangeScoringMode(ScoringMode.PODIUM),
            new CommandBaseScoringPosition(m_base), 
            new CommandPivotScoringPosition(m_pivot),
            // new CommandIndexReverse(m_intake),
            // new WaitCommand(0.20),
            //   new ParallelCommandGroup( 
            //     new CommandShooterStart(m_SAT),
            //     new CommandIndexStop(m_intake)
            // ),
            // new CommandShooterStart(m_SAT),
            new CommandIndexStart(m_index),
            new WaitCommand(0.3),
            new ParallelCommandGroup( 
                new CommandIndexStop(m_index),
                new CommandShooterStop(m_shooter)
            ),
            new CommandChangeRobotHasNote(false)
        );
    }
}