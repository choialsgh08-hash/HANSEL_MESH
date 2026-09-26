from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def text(rel):
    return (ROOT / rel).read_text()


def test_normal_role_start_does_not_enable_temp_internet():
    s = text('scripts/start_role_network.sh')
    assert 'enable_temp_internet' not in s
    assert 'setup_base_routing.sh' in s
    assert 'setup_mesh_route_to_laptop.sh' in s


def test_normal_base_path_has_no_nat_or_default_route_add():
    s = text('scripts/setup_base_routing.sh')
    assert 'masquerade' not in s.lower()
    assert 'nft add' not in s.lower()
    assert 'ip route replace default' not in s


def test_normal_mesh_node_path_has_no_default_route_add():
    s = text('scripts/setup_mesh_route_to_laptop.sh')
    assert 'ip route replace default' not in s
    assert '192.168.60.0/24' in s


def test_nat_is_isolated_to_explicit_temp_helper():
    s = text('scripts/enable_temp_internet_base.sh')
    assert 'masquerade' in s.lower()
    assert 'hansel_temp_internet' in s
    assert 'ip route replace default' in s


def test_temp_nat_is_not_persisted():
    for rel in ('scripts/enable_temp_internet_base.sh', 'scripts/setup_base_routing.sh'):
        s = text(rel)
        assert '/etc/nftables.conf' not in s


def test_systemd_normal_path_cannot_call_temp_internet_helpers():
    service = text('services/hansel-mesh@.service')
    role = text('scripts/start_role_network.sh')
    assert 'start_role_network.sh' in service
    assert 'enable_temp_internet' not in service
    assert 'enable_temp_internet' not in role


def test_role_network_uses_only_routing_helpers():
    role = text('scripts/start_role_network.sh')
    assert 'setup_base_routing.sh' in role
    assert 'setup_mesh_route_to_laptop.sh' in role
    assert 'setup_base_gateway.sh' not in role
