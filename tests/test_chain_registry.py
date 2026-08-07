from __future__ import annotations

import unittest

import _paths  # noqa: F401
from hansel_operator.chain_registry import ChainRegistry, roles_from_entries


def make_registry() -> ChainRegistry:
    return ChainRegistry(
        ordered_units=["head", "node1", "node2", "node3"],
        roles={
            "head": "head",
            "node1": "rear",
            "node2": "rear",
            "node3": "rear",
        },
        active_drive_units=["head", "node1", "node2", "node3"],
    )


class ChainRegistryTests(unittest.TestCase):
    def test_actuator_is_previous_chain_unit(self) -> None:
        cases = [("node1", "head"), ("node2", "node1"), ("node3", "node2")]
        for released, actuator in cases:
            with self.subTest(released=released):
                self.assertEqual(make_registry().actuator_for(released), actuator)

    def test_head_cannot_be_released(self) -> None:
        with self.assertRaisesRegex(ValueError, "head"):
            make_registry().actuator_for("head")

    def test_ack_transition_removes_only_requested_unit(self) -> None:
        registry = make_registry()
        registry.mark_relay_assumed("node3")
        self.assertEqual(registry.active_drive_units, ["head", "node1", "node2"])
        self.assertEqual(registry.relay_assumed_units, ["node3"])

    def test_relay_assumed_unit_cannot_be_reactivated(self) -> None:
        registry = make_registry()
        registry.mark_relay_assumed("node2")
        with self.assertRaisesRegex(ValueError, "RELAY_ASSUMED"):
            registry.set_active(["head", "node1", "node2", "node3"])

    def test_roles_entries_are_strict(self) -> None:
        self.assertEqual(
            roles_from_entries(["head=head", "node1=rear"]),
            {"head": "head", "node1": "rear"},
        )
        with self.assertRaises(ValueError):
            roles_from_entries(["node1"])


if __name__ == "__main__":
    unittest.main()
