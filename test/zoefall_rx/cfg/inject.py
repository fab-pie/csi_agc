import re
import config
from common.data.component import Component
from common.data.cfg import Cfg
from common.data.device import Device
from common.utils.io import replace_matching_line


def _warn_if_sensitive_logs_enabled() -> bool:
    """Return True = ok to proceed, False = user aborted."""
    header = config.COMPONENTS_PATH / "zoefall_common" / "include" / "zc_debug.h"
    if not header.exists():
        return True
    m = re.search(r"^\s*#define\s+ZC_LOGS_SENSITIVE\s+(\d+)", header.read_text(encoding="utf-8"), re.MULTILINE)
    if not m or m.group(1) == "0":
        return True
    print()
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║  WARNING: SENSITIVE LOGS ENABLED (ZC_LOGS_SENSITIVE=1)           ║")
    print("║                                                                  ║")
    print("║  This firmware will print in plaintext on the serial monitor:    ║")
    print("║    ThingsBoard token  |  Wi-Fi passwords  |  OTA API key         ║")
    print("║                                                                  ║")
    print("║  To disable: set ZC_LOGS_SENSITIVE 0 in                          ║")
    print("║    firmware/components/zoefall_common/include/zc_debug.h         ║")
    print("║    then rebuild with `idf.py fullclean build`.                   ║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()
    answer = input("Flash anyway? [y/N] ").strip().lower()
    return answer in ("y", "yes")


def inject(component: Component, cfg: Cfg, *devices: Device) -> int:
    def set_cfg(key: str, value: str) -> None:
        replace_matching_line(
            cfg_path,
            lambda line: line.startswith(key + "="),
            lambda _: f'{key}="{value}"',
        )

    if not _warn_if_sensitive_logs_enabled():
        return 1

    rx_dev = next((d for d in devices if d.name.startswith("RX-")), None)
    tx_dev = next((d for d in devices if d.name.startswith("TX-")), None)

    if not rx_dev:
        print("inject (zoefall_rx): no RX device found in device list")
        return 1

    cfg_path = component.cfg() / cfg.value

    set_cfg("CONFIG_ZC_ZF_RX_TB_ACCESS_TOKEN", rx_dev.access_token)

    if tx_dev:
        mac_suffix = f"{tx_dev.mac.value[4]:02X}{tx_dev.mac.value[5]:02X}"
        set_cfg("CONFIG_ZC_ZF_WIFI_AP_SSID", f"ZoeFall{mac_suffix}")

    return 0
