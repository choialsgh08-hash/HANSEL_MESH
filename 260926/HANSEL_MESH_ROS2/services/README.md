# Services

The systemd templates start the role-specific BATMAN-adv network and metrics
agent. `scripts/enable_mesh_autostart.sh` substitutes the `@HANSEL_REPO_DIR@` placeholder with the actual repository
installation path before installing them into `/etc/systemd/system/`.
