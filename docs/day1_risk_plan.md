# Day1 Risk Plan

## 1. wlan0 Mesh 전환 시 SSH 끊김

### 원인

Raspberry Pi의 `wlan0`를 일반 Wi-Fi 연결에서 BATMAN Mesh용 인터페이스로 전환하면 기존 SSH 연결이 끊긴다.

### 대책

Day1에서는 Base Pi만 노트북과 관리망으로 연결한다.

권장 구조:

```text
노트북
  |
Base Pi 관리망: eth0 또는 usb0
Base Pi Mesh망: wlan0 / bat0
  |
Node / Head Mesh망: wlan0 / bat0