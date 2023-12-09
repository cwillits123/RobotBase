from enum import Enum, auto
from commands2 import SubsystemBase
from wpilib import PneumaticsModuleType, Solenoid
from ctre import ControlMode
from util.ctrecheck import ctreCheckError
from util.simfalcon import createMotor
import constants


class IntakeSubsystem(SubsystemBase):
    class IntakeMode(Enum):
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
        self.state = self.IntakeMode.Ret

    def period(self) -> None:
        if self.state == self.IntakeMode.Dep:
            self.intakeMotor.set(
                ControlMode.Velocity,
                constants.kIntakeMotorSpeed
                * constants.kIntakeGearRatio
                * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.IntakeMode.Rev:
            self.intakeMotor.set(
                ControlMode.Velocity,
                -constants.kIntakeMotorSpeed
                * constants.kIntakeGearRatio
                * constants.kTalonVelocityPerRPM,
            )
        elif self.state == self.IntakeMode.Ret:
            self.intakeMotor.set(
                ControlMode.Velocity, 0
            )

    def revIntake(self) -> None:
        self.state = self.IntakeMode.Rev
    
    def retIntake(self) -> None:
        self.state = self.IntakeMode.Ret
    
    def depIntake(self) -> None:
        self.state = self.IntakeMode.Dep
