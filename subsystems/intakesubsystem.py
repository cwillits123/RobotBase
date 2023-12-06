from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import PneumaticsModuleType, Solenoid
from ctre import ControlMode
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IntakeSubsystem(SubsystemBase):
    class Mode(Enum):
        Deployed = auto()
        Retracted = auto()
        Reversed = auto()

    def __init__(self):
        SubsystemBase.__init__(self)
        self.setName(__class__.__name__)

        self.intakeMotor = createMotor(
            constants.kIntakeCANID,
            constants.kCANivoreName,
        )
        print(f"Initalizing: {constants.kIntakeMotorName}")
        if not ctreCheckError(
            "configFactoryDefault",
            self.intakeMotor.configFactoryDefault(constants.kConfigurationTimeoutLimit),
        ):
            return
        if not ctreCheckError(
            "config_kP",
            self.intakeMotor.config_kP(
                constants.kIntakePIDSlot,
                constants.kIntakePGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return

