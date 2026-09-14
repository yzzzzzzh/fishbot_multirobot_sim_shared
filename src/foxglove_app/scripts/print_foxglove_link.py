#!/usr/bin/env python3
from __future__ import annotations

"""
Print a Foxglove Studio shareable link for the running foxglove_bridge.
"""

import argparse
import os
import urllib.parse


def build_link(host: str, port: int, layout_id: str | None, layout_url: str | None, open_in: str | None) -> str:
    params: dict[str, str] = {
        "ds": "foxglove-websocket",
        "ds.url": f"ws://{host}:{port}",
    }
    if layout_id:
        params["layoutId"] = layout_id
    if layout_url:
        params["layoutUrl"] = layout_url
    if open_in:
        params["openIn"] = open_in

    query = urllib.parse.urlencode(params, quote_via=urllib.parse.quote)
    return f"https://app.foxglove.dev/~/view?{query}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a Foxglove Studio shareable link.")
    parser.add_argument("--host", default=os.getenv("FOXGLOVE_PUBLIC_HOST", "localhost"), help="Public host to embed in the link")
    parser.add_argument("--port", type=int, default=int(os.getenv("FOXGLOVE_PORT", "8765")), help="foxglove_bridge port")
    parser.add_argument("--layout-id", default=os.getenv("FOXGLOVE_LAYOUT_ID"), help="Optional saved layout ID")
    parser.add_argument("--layout-url", default=os.getenv("FOXGLOVE_LAYOUT_URL"), help="Optional layout URL to load")
    parser.add_argument("--open-in", default=os.getenv("FOXGLOVE_OPEN_IN"), help="Set to 'desktop' to suggest the desktop app")

    args = parser.parse_args()
    link = build_link(args.host, args.port, args.layout_id, args.layout_url, args.open_in)

    print()
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print("[foxglove] Open this link in your browser or Foxglove Studio:")
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print(urllib.parse.unquote(link))
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    print()


if __name__ == "__main__":
    main()
