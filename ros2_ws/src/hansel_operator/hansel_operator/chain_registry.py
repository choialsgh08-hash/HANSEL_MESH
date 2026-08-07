"""ROS-independent variable-chain registry."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ChainRegistry:
    ordered_units: list[str]
    roles: dict[str, str]
    active_drive_units: list[str]
    relay_assumed_units: list[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.ordered_units = list(self.ordered_units)
        self.active_drive_units = list(self.active_drive_units)
        self.relay_assumed_units = list(self.relay_assumed_units)
        if not self.ordered_units:
            raise ValueError("ordered_units must not be empty")
        if len(set(self.ordered_units)) != len(self.ordered_units):
            raise ValueError("ordered_units must be unique")
        if self.ordered_units[0] != "head":
            raise ValueError("the first ordered unit must be head")
        unknown_roles = set(self.ordered_units) - set(self.roles)
        if unknown_roles:
            raise ValueError(f"roles missing for: {sorted(unknown_roles)}")
        invalid_roles = {
            unit: role for unit, role in self.roles.items() if role not in {"head", "rear"}
        }
        if invalid_roles:
            raise ValueError(f"invalid roles: {invalid_roles}")
        self._ensure_subset(self.active_drive_units, "active_drive_units")
        self._ensure_subset(self.relay_assumed_units, "relay_assumed_units")
        overlap = set(self.active_drive_units) & set(self.relay_assumed_units)
        if overlap:
            raise ValueError(f"active and relay sets overlap: {sorted(overlap)}")

    def _ensure_subset(self, values: list[str], label: str) -> None:
        unknown = set(values) - set(self.ordered_units)
        if unknown:
            raise ValueError(f"{label} contains unknown units: {sorted(unknown)}")

    def actuator_for(self, released_unit_id: str) -> str:
        if released_unit_id not in self.ordered_units:
            raise ValueError(f"unknown released unit: {released_unit_id}")
        index = self.ordered_units.index(released_unit_id)
        if index == 0:
            raise ValueError("head cannot be selected as a released rear unit")
        return self.ordered_units[index - 1]

    def mark_relay_assumed(self, released_unit_id: str) -> None:
        if released_unit_id not in self.active_drive_units:
            raise ValueError(f"unit is not active: {released_unit_id}")
        self.active_drive_units.remove(released_unit_id)
        if released_unit_id not in self.relay_assumed_units:
            self.relay_assumed_units.append(released_unit_id)

    def set_active(self, active_drive_units: list[str]) -> None:
        self._ensure_subset(active_drive_units, "active_drive_units")
        forbidden = set(active_drive_units) & set(self.relay_assumed_units)
        if forbidden:
            raise ValueError(f"RELAY_ASSUMED units cannot be active: {sorted(forbidden)}")
        ordered = [unit for unit in self.ordered_units if unit in active_drive_units]
        self.active_drive_units = ordered

    def snapshot(self) -> tuple[list[str], list[str], list[str]]:
        return (
            list(self.ordered_units),
            list(self.active_drive_units),
            list(self.relay_assumed_units),
        )


def roles_from_entries(entries: list[str]) -> dict[str, str]:
    roles: dict[str, str] = {}
    for entry in entries:
        unit, separator, role = entry.partition("=")
        if not separator or not unit or not role:
            raise ValueError(f"role entry must be unit=role: {entry!r}")
        roles[unit.strip()] = role.strip()
    return roles

