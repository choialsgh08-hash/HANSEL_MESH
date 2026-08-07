# DDS / bat0 boundary

`cyclonedds.xml`은 DDS가 `bat0`만 사용하도록 하는 시작점이다.
최종 discovery peer/server topology와 multicast 정책은 Network 팀의 Mesh
시험 후 확정한다.

예:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

이 파일은 BATMAN-adv, bat0 생성, IP routing을 수행하지 않는다. 해당 설정은
기존 HANSEL_MESH systemd/network 스크립트의 책임이다.

