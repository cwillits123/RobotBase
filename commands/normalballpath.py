from commands2 import ParallelCommandGroup
from commands.indexer.holdball import HoldBall
from commands.intake.deployintake import DeployIntake

from subsystems.indexersubsystem import IndexerSubsystem
from subsystems.intakesubsystem import IntakeSubsystem


class NormalBallPath(ParallelCommandGroup):
    def __init__(self, intake: IntakeSubsystem, indexer: IndexerSubsystem):
        super().__init__(HoldBall(indexer), DeployIntake(intake))
        self.setName(__class__.__name__)