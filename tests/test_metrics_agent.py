import unittest
from unittest import mock

from monitor import metrics_agent


class MetricsInterfaceTests(unittest.TestCase):
    def test_collect_real_uses_configured_batman_interface(self) -> None:
        commands = []

        def fake_run(command):
            commands.append(command)
            return ""

        with mock.patch.object(metrics_agent, "_run", side_effect=fake_run):
            snapshot = metrics_agent.collect_real(
                "head",
                "mesh9",
                "bat9",
                [],
                123.0,
            )

        self.assertIn(
            ["ip", "neigh", "show", "dev", "bat9"],
            commands,
        )
        self.assertIn(["batctl", "-m", "bat9", "o"], commands)
        self.assertIn(["batctl", "-m", "bat9", "n"], commands)
        self.assertEqual(snapshot["mesh_if"], "mesh9")
        self.assertEqual(snapshot["bat_if"], "bat9")

    def test_collect_real_uses_bounded_parallel_ping_commands(self) -> None:
        commands = []

        def fake_run(command):
            commands.append(command)
            return ""

        with mock.patch.object(metrics_agent, "_run", side_effect=fake_run):
            metrics_agent.collect_real(
                "head",
                "wlan0",
                "bat0",
                ["base", "node1", "unknown"],
                123.0,
            )

        self.assertIn(
            ["ping", "-c", "3", "-W", "1", "192.168.50.1"],
            commands,
        )
        self.assertIn(
            ["ping", "-c", "3", "-W", "1", "192.168.50.11"],
            commands,
        )
        self.assertFalse(
            any(command[-1] == "unknown" for command in commands)
        )


if __name__ == "__main__":
    unittest.main()
