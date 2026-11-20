"""Wi-Fi 업링크(server_forward) 시험 (ROS 없이). 서버는 이 시험 안에 띄운 가짜 HTTP 서버"""
import copy
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from odor_drone import server_forward as sf  # noqa: E402

ROOT = Path(__file__).resolve().parents[4]
EXAMPLE = json.loads((ROOT / "interfaces/examples/odor_sample.json").read_text(encoding="utf-8"))


def test_odor_sample_to_server_records():
    rows = sf.to_server_records(EXAMPLE)
    assert [r["kind"] for r in rows] == ["odor", "battery"]
    odor, bat = rows
    assert odor["sensor_id"] == "DX-01" and odor["ts"] == EXAMPLE["stamp"]
    assert odor["nh3_ppb"] == 830.0 and odor["h2s_ppb"] == 21.0 and odor["so2_ppb"] is None
    assert (odor["lat"], odor["lon"], odor["alt_m"]) == (35.8243715, 127.1486021, 20.1)
    assert bat["drone_id"] == "DRN-01" and bat["voltage_v"] == 23.4


def test_warmup_and_simulated_are_filtered_unless_dry_run():
    warm = copy.deepcopy(EXAMPLE)
    warm["gas"]["flags"] = sf.FLAGS["WARMUP"]
    assert [r["kind"] for r in sf.to_server_records(warm)] == ["battery"]
    fake = copy.deepcopy(EXAMPLE)
    fake["gas"]["flags"] = sf.FLAGS["SIMULATED"]
    assert sf.to_server_records(fake) == []
    assert len(sf.to_server_records(fake, send_all=True)) == 2


def test_no_position_no_voltage():
    rec = copy.deepcopy(EXAMPLE)
    rec["drone"].update(latitude=None, longitude=None, battery_voltage=None)
    rows = sf.to_server_records(rec)
    assert len(rows) == 1 and "lat" not in rows[0]


class _Server:
    """POST 를 받아 두고, status 와 rejected 를 정해 둔 대로 돌려주는 가짜 적재 API"""

    def __init__(self):
        self.status, self.reject_index, self.calls = 200, [], []
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                outer.calls.append((self.path, self.headers.get("X-Device-Token"), body["records"]))
                n = len(body["records"])
                resp = {"received": n, "accepted": n - len(outer.reject_index), "duplicates": 0,
                        "rejected": [{"index": i, "error": "등록되지 않은 장비"} for i in outer.reject_index]}
                data = json.dumps(resp).encode()
                self.send_response(outer.status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *a):
                pass

        self.httpd = HTTPServer(("127.0.0.1", 0), H)
        self.url = f"http://127.0.0.1:{self.httpd.server_port}/api/ingest"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()


def test_forwarder_sends_keeps_and_rejects(tmp_path):
    srv = _Server()
    try:
        fwd = sf.Forwarder(srv.url, "tok", tmp_path, dry_run=True)
        fwd.submit(EXAMPLE)
        assert fwd.pending() == 2

        srv.status = 401                                   # 토큰 문제: spool 에 남긴다
        assert fwd.flush_once() is False and fwd.pending() == 2

        srv.status, srv.reject_index = 200, [1]            # 받음. 1 건은 거절되어 rejected 파일로
        assert fwd.flush_once() is True and fwd.pending() == 0
        path, token, records = srv.calls[-1]
        assert path.endswith("?dry_run=1") and token == "tok" and len(records) == 2
        rej = [json.loads(ln) for ln in (tmp_path / "forward_rejected.jsonl").read_text().splitlines()]
        assert rej[0]["record"]["kind"] == "battery"
    finally:
        srv.close()


def test_forwarder_unreachable_keeps_spool(tmp_path):
    fwd = sf.Forwarder("http://127.0.0.1:9/api/ingest", "tok", tmp_path, timeout_s=1.0)
    fwd.submit(EXAMPLE)
    assert fwd.flush_once() is False and fwd.pending() == 2   # Wi-Fi 끊김과 같은 경우. 버리지 않는다
