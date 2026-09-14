#!/usr/bin/env python3
import argparse
import json
from copy import deepcopy
from pathlib import Path

TOKEN = "{{BOT}}"

def subst(obj, bot: str):
    if isinstance(obj, str):
        return obj.replace(TOKEN, bot)
    if isinstance(obj, list):
        return [subst(x, bot) for x in obj]
    if isinstance(obj, dict):
        out = {}
        for k, v in obj.items():
            out[subst(k, bot)] = subst(v, bot)
        return out
    return obj

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bots", nargs="+", required=True, help='Robot names, e.g. bot1 bot2 bot3')
    ap.add_argument("--backbone", default="../layouts/template/backbone.json")
    ap.add_argument("--template", default="../layouts/template/tab_template.json")
    ap.add_argument("--out", default="../layouts/foxglove_multi_robot_tabs.json")
    args = ap.parse_args()

    backbone = json.loads(Path(args.backbone).read_text())
    tab_tmpl = json.loads(Path(args.template).read_text())

    tab_panel_id = "Tab!robots"
    if tab_panel_id not in backbone["configById"]:
        raise SystemExit(f"backbone.json must contain configById.{tab_panel_id}")

    tabs = backbone["configById"][tab_panel_id].get("tabs", [])
    cfg = backbone["configById"]

    for bot in args.bots:
        inst = subst(deepcopy(tab_tmpl), bot)
        tab_obj = inst["tab"]
        panels = inst["panels"]

        for pid in panels.keys():
            if pid in cfg:
                raise SystemExit(f"Panel id collision: {pid}")

        cfg.update(panels)
        tabs.append(tab_obj)

    backbone["configById"][tab_panel_id]["tabs"] = tabs
    Path(args.out).write_text(json.dumps(backbone, indent=2))
    print(f"Wrote: {args.out}")

if __name__ == "__main__":
    main()
