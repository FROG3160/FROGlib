"""Base subsystem abstractions and dashboard integration for FROGlib.

Provides the `FROGSubsystem` base class, extending WPILib's `commands2.Subsystem`
with automated NetworkTables telemetry publishing, dashboard tunables, and WPILib
DataLogManager logging via descriptors and decorators.
"""

from typing import Any, Callable, Optional, Type, Dict
import ntcore
from commands2 import Subsystem
from wpilib import DataLogManager
from wpiutil.log import DoubleLogEntry, BooleanLogEntry, StringLogEntry
from wpimath.geometry import Pose2d
from enum import Enum


class Direction(Enum):
    """Enumeration representing motor or mechanism movement direction."""

    FORWARD = 1
    REVERSE = -1
    IDLE = 0


class Tunable:
    """Descriptor for tunable NetworkTables values editable from dashboard or code.

    Enables bidirectional synchronization of numeric, boolean, string, or Pose2d values
    with NetworkTables under the subsystem's `tunables` sub-table. Changes made on the
    dashboard automatically invoke any registered setter callback.
    """

    def __init__(
        self,
        default: float | bool | str,
        topic_name: Optional[str] = None,
        setter: Optional[Callable[[Any, Any], None]] = None,
    ):
        """Initializes the Tunable descriptor.

        Args:
            default (float | bool | str): Default value published to NetworkTables.
            topic_name (Optional[str], optional): Custom NetworkTables topic name.
                Defaults to None (uses attribute name).
            setter (Optional[Callable[[Any, Any], None]], optional): Optional callback
                invoked as `setter(subsystem_instance, new_value)` whenever the value changes.
        """
        self.default = default
        self.topic_name = topic_name
        self.setter = setter
        self._attr_name = ""

    def __set_name__(self, owner, name):
        """Captures the attribute name on the owning class.

        Args:
            owner (type): The class owning this descriptor.
            name (str): The attribute name assigned to this descriptor.
        """
        self._attr_name = name
        if self.topic_name is None:
            self.topic_name = name

    def _get_storage(self, obj: "FROGSubsystem") -> Dict[str, Any]:
        """Retrieves or initializes the NetworkTables publisher and subscriber storage.

        Args:
            obj (FROGSubsystem): The subsystem instance owning this attribute.

        Returns:
            Dict[str, Any]: Storage dictionary with "value", "pub", and "sub".
        """
        if not hasattr(obj, "_tunable_data"):
            setattr(obj, "_tunable_data", {})
        tunable_data = getattr(obj, "_tunable_data")
        if self._attr_name not in tunable_data:
            topic = self._get_typed_topic(
                obj.nt_tunables, self.topic_name, type(self.default)
            )
            pub = topic.publish()
            pub.set(self.default)
            sub = topic.subscribe(self.default)
            tunable_data[self._attr_name] = {
                "value": self.default,
                "pub": pub,
                "sub": sub,
            }
        return tunable_data[self._attr_name]

    def __get__(self, obj: "FROGSubsystem", objtype: Optional[type] = None) -> Any:
        """Gets the current tunable value, polling NetworkTables for dashboard updates.

        Args:
            obj: The subsystem instance.
            objtype: The owner class type.

        Returns:
            Current tunable value, or the descriptor instance if accessed on class.
        """
        if obj is None:
            return self
        storage = self._get_storage(obj)
        # Update local value from NetworkTables if it changed
        new_val = storage["sub"].get()
        if new_val != storage["value"]:
            storage["value"] = new_val
            # Call the decorated setter method if it exists
            if self.setter:
                self.setter(obj, new_val)
            # Legacy check for _set_ prefix
            setter_name = f"_set_{self._attr_name}"
            if hasattr(obj, setter_name):
                getattr(obj, setter_name)(new_val)
        return storage["value"]

    def __set__(self, obj: "FROGSubsystem", value):
        """Sets the tunable value, publishing the new value to NetworkTables.

        Args:
            obj (FROGSubsystem): The subsystem instance.
            value (Any): The new value to set and publish.
        """
        storage = self._get_storage(obj)
        storage["value"] = value
        storage["pub"].set(value)
        # Call the decorated setter method if it exists
        if self.setter:
            self.setter(obj, value)
        # Legacy check for _set_ prefix
        setter_name = f"_set_{self._attr_name}"
        if hasattr(obj, setter_name):
            getattr(obj, setter_name)(value)

    def _get_typed_topic(self, table: ntcore.NetworkTable, key: str, value_type: type):
        """Returns the appropriate typed NetworkTables topic for the given value type.

        Args:
            table (ntcore.NetworkTable): NetworkTables table to create topic in.
            key (str): Topic key name.
            value_type (type): Python type (int, float, bool, str, or Pose2d).

        Returns:
            Optional[ntcore.Topic]: Typed topic object or None if unsupported.
        """
        if value_type in (int, float):
            return table.getDoubleTopic(key)
        elif value_type is bool:
            return table.getBooleanTopic(key)
        elif value_type is str:
            return table.getStringTopic(key)
        elif value_type is Pose2d:
            return table.getStructTopic(key, Pose2d)
        else:
            return None


class Telemetry:
    """Descriptor for read-only telemetry values published to NetworkTables and WPILib DataLog.

    Wraps a getter method to periodically publish its return value to NetworkTables under
    the subsystem's `telemetry` sub-table, and optionally logs historical entries to
    WPILib DataLogManager.
    """

    def __init__(
        self,
        getter: Callable[[Any], Any],
        topic_name: Optional[str] = None,
        log: bool = True,
    ):
        """Initializes the Telemetry descriptor.

        Args:
            getter (Callable[[Any], Any]): Getter method returning the telemetry value.
            topic_name (Optional[str], optional): Custom topic name. Defaults to None (uses method name).
            log (bool, optional): Whether to record values to WPILib DataLog. Defaults to True.
        """
        self.getter = getter
        self.topic_name = topic_name
        self.log = log
        self._attr_name = ""

    def __set_name__(self, owner, name):
        """Captures the attribute name on the owning class.

        Args:
            owner (type): The class owning this descriptor.
            name (str): The attribute name assigned to this descriptor.
        """
        self._attr_name = name
        if self.topic_name is None:
            self.topic_name = name

    def _get_storage(self, obj: "FROGSubsystem") -> Dict[str, Any]:
        """Retrieves or initializes the NetworkTables publisher and DataLog entry.

        Args:
            obj (FROGSubsystem): The subsystem instance.

        Returns:
            Dict[str, Any]: Storage dictionary with "pub" and "log".
        """
        if not hasattr(obj, "_telemetry_data"):
            setattr(obj, "_telemetry_data", {})
        telemetry_data = getattr(obj, "_telemetry_data")
        if self._attr_name not in telemetry_data:
            # We need a sample value to determine type
            value = self.getter(obj)
            topic = self._get_typed_topic(obj.nt_table, self.topic_name, type(value))

            pub = None
            if topic:
                pub = topic.publish()

            log_entry = None
            if self.log:
                path = f"FROGSubsystems/{obj.getName()}/telemetry/{self.topic_name}"
                if isinstance(value, (int, float)):
                    log_entry = DoubleLogEntry(DataLogManager.getLog(), path)
                elif isinstance(value, bool):
                    log_entry = BooleanLogEntry(DataLogManager.getLog(), path)
                elif isinstance(value, str):
                    log_entry = StringLogEntry(DataLogManager.getLog(), path)

            telemetry_data[self._attr_name] = {"pub": pub, "log": log_entry}
        return telemetry_data[self._attr_name]

    def __get__(self, obj: "FROGSubsystem", objtype: Optional[type] = None) -> Any:
        """Evaluates getter, publishes to NetworkTables, appends to DataLog, and returns value.

        Args:
            obj: The subsystem instance.
            objtype: The owner class type.

        Returns:
            Evaluated telemetry value, or the descriptor instance if accessed on class.
        """
        if obj is None:
            return self
        value = self.getter(obj)
        storage = self._get_storage(obj)
        if storage["pub"]:
            storage["pub"].set(value)
        if storage["log"]:
            storage["log"].append(value)
        return value

    def _get_typed_topic(self, table: ntcore.NetworkTable, key: str, value_type: type):
        """Returns the appropriate typed NetworkTables topic for the given value type.

        Args:
            table (ntcore.NetworkTable): NetworkTables table to create topic in.
            key (str): Topic key name.
            value_type (type): Python type (int, float, bool, str, or Pose2d).

        Returns:
            Optional[ntcore.Topic]: Typed topic object or None if unsupported.
        """
        if value_type in (int, float):
            return table.getDoubleTopic(key)
        elif value_type is bool:
            return table.getBooleanTopic(key)
        elif value_type is str:
            return table.getStringTopic(key)
        elif value_type is Pose2d:
            return table.getStructTopic(key, Pose2d)
        else:
            return None


def tunable(
    default: float | bool | str, topic_name: Optional[str] = None
) -> Callable[[Callable[[Any, Any], None]], Tunable]:
    """Decorator for a method to act as a setter for a tunable NetworkTables value.

    Args:
        default: Initial default value.
        topic_name: NetworkTables topic name. Defaults to None (uses decorated method name).

    Returns:
        Decorator producing a Tunable descriptor.

    Example:
        ```python
        @tunable(0.5)
        def my_value(self, val):
            self.hardware.set(val)
        ```
    """

    def decorator(func: Callable[[Any, Any], None]):
        return Tunable(default, topic_name, setter=func)

    return decorator


def telemetry(
    topic_name: Optional[str] = None, log: bool = True
) -> Callable[[Callable[[Any], Any]], Telemetry]:
    """Decorator for a method to act as an automated telemetry getter.

    Args:
        topic_name: NetworkTables topic name. Defaults to None (uses decorated method name).
        log: Whether to record values to WPILib DataLog. Defaults to True.

    Returns:
        Decorator producing a Telemetry descriptor.

    Example:
        ```python
        @telemetry()
        def motor_velocity(self) -> float:
            return self.motor.get_velocity().value
        ```
    """

    def decorator(func: Callable[[Any], Any]):
        return Telemetry(func, topic_name, log)

    return decorator


class FROGSubsystem(Subsystem):
    """Base class for FROG Team subsystems with automatic NT tunables & telemetry.

    Inherit from this instead of WPILib's plain `commands2.Subsystem` to gain
    automatic telemetry discovery, dashboard tuning, and DataLog integration.
    """

    # Expose as class attributes for decorator access
    tunable: Callable[..., Callable[..., Tunable]] = tunable
    telemetry: Callable[..., Callable[..., Telemetry]] = telemetry

    def __init__(self, *args, **kwargs):
        """Initializes the FROGSubsystem and discovers telemetry and tunable attributes."""
        super().__init__(*args, **kwargs)
        # Ensure name is set if not already set by Subsystem
        if not self.getName():
            self.setName(self.__class__.__name__)

        nt = ntcore.NetworkTableInstance.getDefault()
        self.nt_root = nt.getTable("FROGSubsystems")
        self.nt_table = self.nt_root.getSubTable(self.getName())
        self.nt_tunables = self.nt_table.getSubTable("tunables")
        self.nt_telemetry = self.nt_table.getSubTable("telemetry")

        # Discover all Telemetry and Tunable attributes once
        self._telemetry_attrs = []
        self._tunable_attrs = []
        discovered_motors = []
        from froglib.ctre import FROGTalonFX

        for name in dir(self.__class__):
            attr = getattr(self.__class__, name)
            if isinstance(attr, Telemetry):
                self._telemetry_attrs.append(name)
            elif isinstance(attr, Tunable):
                self._tunable_attrs.append(name)

        # Also discover motors on the instance
        for attr_name, attr_value in self.__dict__.items():
            if isinstance(attr_value, FROGTalonFX):
                discovered_motors.append(attr_value)
            elif isinstance(attr_value, (list, tuple)):
                for item in attr_value:
                    if isinstance(item, FROGTalonFX):
                        discovered_motors.append(item)
                    # Support list of SwerveModules
                    elif hasattr(item, "drive_motor") and isinstance(
                        item.drive_motor, FROGTalonFX
                    ):
                        discovered_motors.append(item.drive_motor)
                    if hasattr(item, "steer_motor") and isinstance(
                        item.steer_motor, FROGTalonFX  # type: ignore
                    ):
                        discovered_motors.append(item.steer_motor)  # type: ignore

        self._motors = discovered_motors

    @telemetry("Current Command", log=True)
    def current_command_telem(self) -> str:
        """Returns the name of the currently running command for telemetry logging."""
        cmd = self.getCurrentCommand()
        return cmd.getName() if cmd else "None"

    @telemetry(".hasCommand", log=False)
    def has_command_telem(self) -> bool:
        """Returns True if the subsystem currently has an active command running."""
        return self.getCurrentCommand() is not None

    @telemetry("Default Command", log=True)
    def default_command_telem(self) -> str:
        """Returns the name of the subsystem's default command for telemetry logging."""
        cmd = self.getDefaultCommand()
        return cmd.getName() if cmd else "None"

    @telemetry(".hasDefault", log=False)
    def has_default_telem(self) -> bool:
        """Returns True if a default command is registered for this subsystem."""
        return self.getDefaultCommand() is not None

    def get_command_name(self) -> str:
        """Returns the name of the currently running command, or an empty string if none.

        Returns:
            str: Name of currently executing command or empty string.
        """
        cmd = self.getCurrentCommand()
        return cmd.getName() if cmd else ""

    def periodic(self) -> None:
        """Automatically updates all telemetry and checks for tunable updates.

        Called once per scheduler run (typically every 20ms).
        """
        for attr_name in self._telemetry_attrs:
            getattr(self, attr_name)  # Triggers Telemetry.__get__

        for attr_name in self._tunable_attrs:
            getattr(
                self, attr_name
            )  # Triggers Tunable.__get__ (which checks for NT updates)
