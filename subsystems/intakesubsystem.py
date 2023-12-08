from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import PneumaticsModuleType, Solenoid
from ctre import ControlMode
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IntakeSubsystem(SubsystemBase):
    class Mode(Enum):
        Dep = auto()
        Ret = auto()
        Rev = auto()

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
        if not ctreCheckError(
            "config_kI",
            self.intakeMotor.config_kI(
                constants.kIntakePIDSlot,
                constants.kIntakeIGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        if not ctreCheckError(
            "config_kD",
            self.intakeMotor.config_kD(
                constants.kIntakePIDSlot,
                constants.kIntakeDGain,
                constants.kConfigurationTimeoutLimit,
            ),
        ):
            return
        print(f"{constants.kIntakeMotorName} Initialization Complete")
        self.state = self.Mode.Ret

    def period(self) -> None:
        if self.state == self.Mode.Dep:
            self.intakeMotor.set(
                ControlMode.Velocity,
                constants.kIntakeSpeed
                * constants.kIntakeGearRatio
                * constants.kTalonVelocityPerRPM,
            )

