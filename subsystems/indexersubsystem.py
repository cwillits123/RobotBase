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
            constants.kIndexerMotorId,
        )


        #For Indexer
        print(f"Initializing IndexerMotor")
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
        print(f"IndexerMotor Initalization Complete")

        #For Staging
        self.stagingMotor = createMotor(
            constants.kStagingMotorId,
        )
        print(f"Initalizing StagingMotor")
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
        if not ctreCheckError(
            "configForwardSwitchLimitSource",
            self.stagingMotor.configForwardLimitSwitchSource(
                LimitSwitchSource.Deactivated,
                LimitSwitchNormal.Disabled,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "configReverseSwitchLimitSource",
            self.stagingMotor.configReverseLimitSwitchSource(
                LimitSwitchSource.Deactivated,
                LimitSwitchNormal.Disabled,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print(f"StagingMotor Initilization Complete")
        self.indexerSensor = self.stagingMotor.isRevLimitSwitchClosed
        self.stagingSensor = self.stagingMotor.isFwdLimitSwitchClosed
        self.state = self.IndexerMode.Holding

    def period(self) -> None:
        SmartDashboard.putBoolean("readyToFire", self.stagingSensor() == 0)
        SmartDashboard.putBoolean("twoBalls", self.stagingSensor() == 0 and self.indexerSensor() == 0)

        if self.state == self.IndexerMode.Feed:
            self.indexerMotor.set(
                ControlMode.Velocity,
                constants.kIndexerMotorSpeed
                * constants.kIndexerMotorGearRatio
                * constants.kTalonVelocityPerRPM,
            )
            self.stagingMotor.set(
                ControlMode.Velocity,
                constants.kStagingMotorSpeed
                * constants.kStagingMotorGearRatio
                * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.IndexerMode.Holding:
            if not (self.indexerSensor() == 0 and self.stagingSensor() == 0):
                self.indexerMotor.set(
                    ControlMode.Velocity,
                    constants.kIndexerMotorSpeed
                    * constants.kIndexerMotorGearRatio
                    * constants.kTalonVelocityPerRPM,
                )
                self.stagingMotor.set(
                    ControlMode.Velocity,
                    -constants.kStagingMotorSpeed
                    * constants.kStagingMotorGearRatio
                    * constants.kTalonVelocityPerRPM,
                )
            else:
                self.indexerMotor.set(ControlMode.Velocity, 0)
                self.stagingMotor.set(ControlMode.Velocity, 0)
        elif self.state == self.IndexerMode.Reversed:
            self.indexerMotor.set(
                ControlMode.Velocity,
                -constants.kIndexerMotorSpeed,
                * constants.kIndexerMotorGearRatio
                * constants.kTalonVelocityPerRPM,
            )
            self.stagingMotor.set(
                ControlMode.Velocity,
                -constants.kStagingMotorSpeed
                * constants.kStagingMotorGearRatio
                * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.IndexerMode.Off:
            self.indexerMotor.neutralOutput()
            self.stagingMotor.neutralOutput()

    def motorsOff(self) -> None:
        self.state = self.IndexerMode.Off
    def motorsReversed(self) -> None:
        self.state = self.IndexerMode.Reversed
    def motorsFeed(self) -> None:
        self.state = self.IndexerMode.Feed
    def motorsHolding(self) -> None:
        self.state = self.IndexerMode.Holding
