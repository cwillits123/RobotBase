from commands2 import WaitCommand
import wpilib
from wpimath.geometry import Pose2d
import commands2
import commands2.button


import constants

from commands.resetdrive import ResetDrive
from commands.indexer.feedforward import FeedForward
from commands.indexer.holdball import HoldBall
from commands.drivedistance import DriveDistance

from commands.reverseballpath import ReverseBallPath
from commands.normalballpath import NormalBallPath
from commands.shootball import ShootBall
from commands.defensestate import DefenseState

from commands.intake.deployintake import DeployIntake
from commands.intake.retractintake import RetractIntake

from subsystems.drivesubsystem import DriveSubsystem
#from subsystems.visionsubsystem import VisionSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.indexersubsystem import IndexerSubsystem

from operatorinterface import OperatorInterface
from util.helpfultriggerwrappers import SmartDashboardButton


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # The operator interface (driver controls)
        self.operatorInterface = OperatorInterface()

        # The robot's subsystems
        self.drive = DriveSubsystem()
        #self.vision = VisionSubsystem()
        self.intake = IntakeSubsystem()
        self.indexer = IndexerSubsystem()


        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = commands2.ParallelCommandGroup(
            commands2.SequentialCommandGroup(
                ResetDrive(self.drive),
                HoldBall(self.indexer),
                DeployIntake(self.intake),
                DriveDistance(
                    4 * constants.kWheelCircumference,
                    constants.kAutoDriveSpeedFactor,
                    DriveDistance.Axis.X,
                    self.drive,
                ),
                RetractIntake(self.intake),
                commands2.WaitCommand(2),
                FeedForward(self.indexer),
                commands2.WaitCommand(2),
                HoldBall(self.indexer),
            ),
            
        )

        # A complex auto routine that drives to the target, drives forward, waits, drives back
    

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        #self.chooser.addOption("Complex Auto", self.complexAuto)
        #self.chooser.addOption("Target Auto", self.driveToTarget)
        #self.chooser.addOption(
            #"2 Ball Left Hanger Outtake Auto", self.twoBLHangerOuttake
        #)
        #self.chooser.addOption("4 Ball Left Noninvasive Auto", self.fourBLNoninvasive)
        #self.chooser.addOption("5 Ball Right Standard Auto", self.fiveBRStandard)
        #self.chooser.addOption("3 Ball Right Standard Auto", self.threeBRStandard)
        #self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)

        # Put the chooser on the dashboard
        #wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

       

       
    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        commands2.button.JoystickButton(
            *self.operatorInterface.deployIntakeControl,
        ).whenHeld(DeployIntake(self.intake)).whenReleased(RetractIntake(self.intake))

        (
            commands2.button.JoystickButton(
                *self.operatorInterface.deployIntakeControl,
            ).and_(
                commands2.button.JoystickButton(
                    *self.operatorInterface.reverseBallPath,
                )
            )
        ).whenActive(ReverseBallPath(self.intake, self.indexer))

        (
            commands2.button.JoystickButton(
                *self.operatorInterface.deployIntakeControl,
            ).and_(
                commands2.button.JoystickButton(
                    *self.operatorInterface.reverseBallPath,
                ).not_()
            )
        ).whenActive(
            NormalBallPath(self.intake, self.indexer)
        )  # when let go of just the reverse button, go back to normal ball path

        

        

       

        commands2.button.JoystickButton(*self.operatorInterface.resetGyro).whenPressed(
            ResetDrive(self.drive, Pose2d(0, 0, 0))
        )

        commands2.button.JoystickButton(
            *self.operatorInterface.defenseStateControl
        ).whileHeld(DefenseState(self.drive))

        

        

        

        commands2.button.JoystickButton(*self.operatorInterface.shootBall).whenHeld(
            ShootBall(self.indexer)
        ).whenReleased(HoldBall(self.indexer))

      
    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()