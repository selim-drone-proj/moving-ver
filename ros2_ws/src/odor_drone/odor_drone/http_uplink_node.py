"""
http_uplink_node
----------------
/odor/sample 을 Wi-Fi(또는 LTE 등 IP 망)로 관제 서버(odor_system)의 적재 API 에 바로 보낸다.
LoRa(lora_uplink_node)와 함께 띄울 수 있다 (real.launch.py uplink:=both).

    POST <url>  헤더 X-Device-Token: <토큰>  본문 {"records": [...]}   (odor_system docs/HARDWARE_DATA_SPEC.md 5-1-a)

- 변환, spool, 재전송은 server_forward.py (지상국 게이트웨이와 같은 코드)가 한다.
  odor_sample 1 건 -> 서버 레코드 odor(DX-01) + battery(DRN-01).
  예열, 영점 보정, 퍼지 중 가스 값과 가짜 데이터(SIMULATED)는 보내지 않는다. dry_run 이면 형식 확인용으로 모두 보낸다.
- 보낼 레코드는 먼저 <spool_dir>/forward_spool.jsonl 에 적고 서버가 받으면 지운다.
  Wi-Fi 가 끊겨도 쌓아 두었다가 다시 붙으면 보낸다. 노드를 다시 켜도 남은 것부터 보낸다.
  서버는 (장비, 시각) 중복을 건너뛰므로 같은 레코드를 다시 보내도 안전하다.
- 서버가 거절한 레코드는 <spool_dir>/forward_rejected.jsonl 에 이유와 함께 남는다.

파라미터: url, dry_run, period_s, spool_dir
토큰은 파라미터 파일에 적지 않고 환경변수 ODOR_INGEST_TOKEN 으로 준다 (launch 하기 전에 export).
"""

import os
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from odor_msgs.msg import OdorSample

from .odor_json import msg_to_dict
from .server_forward import Forwarder

PERIOD_S = 5.0
SPOOL_DIR = "~/odor_logs/uplink"


class HttpUplinkNode(Node):
    def __init__(self):
        super().__init__("http_uplink_node")
        url = str(self.declare_parameter("url", "").value).strip()
        dry_run = bool(self.declare_parameter("dry_run", True).value)
        period = float(self.declare_parameter("period_s", PERIOD_S).value)
        spool_dir = Path(os.path.expanduser(str(self.declare_parameter("spool_dir", SPOOL_DIR).value)))
        if not url:
            raise RuntimeError("http_uplink_node: url 파라미터가 비어 있다 (예: http://<서버>:8090/api/ingest)")
        token = os.environ.get("ODOR_INGEST_TOKEN", "")
        if not token:
            self.get_logger().warn("ODOR_INGEST_TOKEN 이 없어 서버가 거절한다 (401/403). spool 에 쌓아 두고 계속 다시 보낸다")

        spool_dir.mkdir(parents=True, exist_ok=True)
        self.fwd = Forwarder(url, token, spool_dir, dry_run=dry_run, period_s=period)
        self.fwd.start()
        self.create_subscription(OdorSample, "/odor/sample", self.on_sample, 10)
        self.create_timer(60.0, self.report)
        self.get_logger().info(f"http_uplink_node 시작: {url}{' (dry_run: 서버는 검증만, 저장 안 함)' if dry_run else ''}, "
                               f"spool {spool_dir}, 밀린 것 {self.fwd.pending()} 건")

    def on_sample(self, msg: OdorSample):
        self.fwd.submit(msg_to_dict(msg, "odor_sample"))

    def report(self):
        self.get_logger().info(self.fwd.status())

    def destroy_node(self):
        self.fwd.close()            # 남은 것을 한 번 더 보내 본다. 못 보낸 것은 spool 에 남는다
        super().destroy_node()


def main():
    rclpy.init()
    node = HttpUplinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
