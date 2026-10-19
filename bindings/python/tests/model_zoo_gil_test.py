"""Model-zoo I/O must allow a Python HTTP server thread to make progress."""

import os
import subprocess
import sys

import pytest


@pytest.mark.parametrize("operation", ["get", "download"])
def test_model_zoo_releases_gil(tmp_path, operation):
    # Run in a child to isolate the global zoo endpoints and timeout a deadlock.
    script = r'''
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import depthai as dai

root = Path(sys.argv[1])
operation = sys.argv[2]
model_bytes = b"local model payload"

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/health":
            body = b"ok"
        elif path == "/download":
            body = json.dumps({
                "hash": "gil-regression",
                "download_links": [base_url + "/model.onnx"],
            }).encode()
        elif path == "/model.onnx":
            body = model_bytes
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass

server = HTTPServer(("127.0.0.1", 0), Handler)
base_url = "http://127.0.0.1:" + str(server.server_port)
thread = threading.Thread(target=server.serve_forever, daemon=True)
thread.start()
dai.modelzoo.setHealthEndpoint(base_url + "/health")
dai.modelzoo.setDownloadEndpoint(base_url + "/download")
description = dai.NNModelDescription(model="luxonis/gil-regression:model", platform="RVC4")
cache = root / "cache"
try:
    if operation == "get":
        path = dai.getModelFromZoo(description, cacheDirectory=cache)
        assert Path(path).read_bytes() == model_bytes
    else:
        models = root / "models"
        models.mkdir()
        description.saveToYamlFile(models / "model.yaml")
        assert dai.downloadModelsFromZoo(models, cacheDirectory=cache)
        assert [p.read_bytes() for p in cache.rglob("model.onnx")] == [model_bytes]
finally:
    server.shutdown()
    thread.join(timeout=2)
    server.server_close()
'''
    env = os.environ.copy()
    env.update({
        "DEPTHAI_ZOO_INTERNET_CHECK": "1",
        "DEPTHAI_ZOO_INTERNET_CHECK_TIMEOUT": "2000",
        "DEPTHAI_TELEMETRY": "0",
        "NO_PROXY": "127.0.0.1,localhost",
        "no_proxy": "127.0.0.1,localhost",
    })
    result = subprocess.run(
        [sys.executable, "-c", script, str(tmp_path), operation],
        capture_output=True,
        text=True,
        timeout=15,
        env=env,
    )
    assert result.returncode == 0, result.stdout + result.stderr
