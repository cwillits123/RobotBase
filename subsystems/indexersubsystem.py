from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import *
from ctre import *
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IndexerSubsystem(SubsystemBase):
    class IndexerMode(Enum):
        Feed = auto()
        Reversed = auto()
        Off = auto()
        Holding = auto()

    def __init__(self) -> None:
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)
        self.indexerMotor = createMotor(
            constants.KIndexerMotorId,
        )


        #For Indexer
        print(f"Initializing: {constants.kIndexerMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.indexerMotor.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.indexerMotor.config_kP(
                constants.kIndexerPIDSlot,
                constants.kIndexerPGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.indexerMotor.config_kI(
                constants.kIndexerPIDSlot,
                constants.kIndexerIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.indexerMotor.config_kD(
                constants.kIndexerPIDSlot,
                constants.kIndexerDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print(f"{constants.kIndexerMotorName} Initalization Complete")

        #For Staging
        self.stagingMotor = createMotor(
            constants.kStagingMotorId,
        )
        print(f"Initalizing: {constants.kStagingMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.stagingMotor.configFactoryDefault(
                constants.kConfigurationTimeoutLimit
            ),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.stagingMotor.config_kP(
               constants.kStagingPIDSlot,
               constants.kStagingPGain,
               constants.kConfigurationTimeoutLimit, 
            ),
        ):
            return
        if not ctreCheckError(
            "config_kI",
            self.stagingMotor.config_kI(
                constants.kStagingPIDSlot,
                constants.kStagingIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.stagingMotor.config_kD(
                constants.kStagingPIDSlot,
                constants.kStagingDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return