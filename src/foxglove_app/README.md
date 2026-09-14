# Fishbot Foxglove Module

Foxglove - visualization wrapper that launches `foxglove_bridge`.

## Shareable link

When you start the `foxglove` compose service, the container prints a Foxglove Studio shareable link, e.g.:

```
https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws://localhost:8765
```

Environment variables you can set before `docker compose up foxglove`:
- `FOXGLOVE_PUBLIC_HOST`: Hostname or IP to use in the link (default `localhost`).
- `FOXGLOVE_LAYOUT_ID`: Optional saved layout ID to append to the link.
- `FOXGLOVE_OPEN_IN`: Set to `desktop` to prompt the desktop app instead of the web app.

You can also generate a link manually from the workspace with:

```bash
python3 src/foxglove_app/scripts/print_foxglove_link.py
```

## Bot namespace selector extension

An optional Foxglove Studio extension lives in `src/foxglove_app/extensions/bot_namespace_selector`. It adds:
- A panel for choosing which `/botX` namespace should be treated as the active robot.
- Topic aliases to expose `/selected/imu`, `/selected/points`, `/selected/odom`, and `/selected/cmd_vel` for whichever robot you pick.

Build it locally (Node 18+):

```bash
cd src/foxglove_app/extensions/bot_namespace_selector
npm install
npm run build
```

Then sideload `dist/extension.js` in Foxglove Studio (**Settings → Extensions → Install extension from local file…**) and use the `foxglove_multi_robot_selected` layout. The gauge panel in that layout is replaced by the selector panel and the virtual joystick now publishes to `/selected/cmd_vel`.
