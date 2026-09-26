# Validation V4

This review retains the ROS 2 Lyrical source-build fixes from V3 and corrects
network policy so normal HANSEL operation is independent from temporary Internet
sharing.

Validated invariants:

- role/systemd startup contains no NAT/masquerade operation;
- role/systemd startup installs no default route via the operator laptop;
- Base normal startup only assigns the wired transit address and enables IPv4 forwarding;
- head/node1/node2 normal startup only route `192.168.60.0/24` via Base;
- temporary NAT/default routes exist only in explicitly named `enable_temp_internet_*` helpers;
- temporary nftables rules are not persisted to `/etc/nftables.conf`;
- normal startup cleans stale temporary Internet state;
- ROS Lyrical, CycloneDDS, serial preflight, and package dependency fixes from V3 remain.
