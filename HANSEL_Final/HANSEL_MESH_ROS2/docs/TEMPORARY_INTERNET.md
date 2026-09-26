# Temporary Internet access (development only)

Normal HANSEL operation does **not** use NAT and does **not** install a default
route through the operator laptop.

Normal topology:

```text
head/node1/node2 192.168.50.x
        |
      bat0
        |
Base 192.168.50.1 <-> eth0 192.168.60.1
                            |
                    operator laptop 192.168.60.2
```

Normal routing requirements:

- Base forwards between `bat0` and `eth0`.
- Mesh nodes route only `192.168.60.0/24` through `192.168.50.1`.
- The laptop has a route for `192.168.50.0/24` through `192.168.60.1`.
- No masquerade/NAT is required for HANSEL operation.
- No mesh node needs a default route through Base during normal operation.

## Temporarily enable Internet

This is only for package downloads, git, apt, etc. The operator laptop must
already be configured to forward/NAT its `192.168.60.0/24` side to Wi-Fi.

On Base:

```bash
sudo ./scripts/enable_temp_internet_base.sh
```

On the mesh node that needs Internet:

```bash
sudo ./scripts/enable_temp_internet_node.sh
```

The Base helper requires `nft`; install it only when needed:

```bash
sudo apt install nftables
```

## Return to normal operation

On the mesh node:

```bash
sudo ./scripts/disable_temp_internet_node.sh
```

On Base:

```bash
sudo ./scripts/disable_temp_internet_base.sh
```

A normal `hansel-mesh@<role>` restart also removes the temporary default route;
Base startup additionally deletes the temporary `hansel_temp_internet` nft table
if it exists. Nothing is written to `/etc/nftables.conf`.
