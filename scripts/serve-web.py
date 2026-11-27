#!/usr/bin/env python3
"""Static server for web/: serves the wasm gzip-compressed with the right
MIME type. Usage: serve-web.py [port] (default $PORT or 8080)."""

import http.server
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "web")


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".wasm": "application/wasm",
        ".js": "text/javascript",
    }

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def send_head(self):
        path = self.translate_path(self.path.split("?")[0])
        gz = path + ".gz"
        if path.endswith(".wasm") and os.path.exists(gz) and "gzip" in self.headers.get("Accept-Encoding", ""):
            f = open(gz, "rb")
            self.send_response(200)
            self.send_header("Content-Type", "application/wasm")
            self.send_header("Content-Encoding", "gzip")
            self.send_header("Content-Length", str(os.path.getsize(gz)))
            self.send_header("X-Uncompressed-Length", str(os.path.getsize(path)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            return f
        return super().send_head()

    def end_headers(self):
        if not self.path.endswith(".wasm"):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get("PORT", "8080"))
    http.server.ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()
