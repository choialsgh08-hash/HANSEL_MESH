# Windows operator-laptop network (normal operation)

This project does not require Internet sharing/NAT for normal HANSEL control.
The wired link to Base is a routed transit network.

Expected addresses:

```text
Windows operator laptop: 192.168.60.2/24
Base eth0:                192.168.60.1/24
Base bat0:                192.168.50.1/24
Head:                     192.168.50.10/24
Node1:                    192.168.50.11/24
Node2:                    192.168.50.12/24
```

On Windows, configure the Ethernet adapter connected to Base as
`192.168.60.2/24`, then add a persistent route to the mesh subnet via Base.
Use the adapter/index appropriate for the actual machine.

Example PowerShell route logic:

```powershell
route -p add 192.168.50.0 mask 255.255.255.0 192.168.60.1
```

No default gateway on the Base-facing Ethernet adapter is required for normal
HANSEL operation. Windows Internet Connection Sharing / NAT is only needed for
temporary package-download access described in `TEMPORARY_INTERNET.md`.

Important DDS note: IPv4 routing alone does not define CycloneDDS discovery
across different L3 subnets. The repository currently binds Pi-side DDS to
`bat0` and intentionally leaves the final peer/discovery policy TBD in
`ros2_ws/src/hansel_bringup/config/dds/README.md`. Do not treat successful ping
between `192.168.60.2` and `192.168.50.x` as proof that cross-subnet ROS 2 DDS
discovery is complete.
