# Merge-to-main gate — `feature/cellular-x1-ppp`

This branch is **validated in firmware but blocked carrier-side**: Hologram's
G3 PGW is not assigning an IP (`AT+CGPADDR` returns `0.0.0.0`); they have
confirmed and escalated it. The firmware PPP bring-up (LCP + IPCP) is correct.

**Do NOT merge to `main` or push until every box below passes.** Keep the work
on this branch.

## Gate (all must pass)

- [ ] **SIM has an IP** — `AT+CGPADDR` returns a real, non-`0.0.0.0` address
      (i.e. Hologram has restored IP assignment on the SIM).
- [ ] **PPP gets an IP end-to-end** — IPCP completes and the log shows
      `PPP got IP: <addr>` on a clean cold boot.
- [ ] **Live detection upload works** — a real
      `POST /api/nodes/WSW-X1-CELL-001/detections` succeeds (2xx) and the node
      shows **online** in the dashboard.

## Must-do before cutting the production build

- [ ] **Remove the lwIP PPP debug flags** from `sdkconfig.defaults`:
      `CONFIG_LWIP_DEBUG` and `CONFIG_LWIP_PPP_DEBUG_ON`. They were enabled
      only for bring-up diagnostics (printf-path LCP visibility) and bloat the
      binary / spam the console.
- [ ] **Decide on static node-position in NVS.** GPS acquisition is
      time-boxed pre-PPP and best-effort, so a stationary node can miss a cold
      fix (TTFF > the window) and upload without `node_position`. If a reliable
      node position is required, add a static lat/lon to NVS (namespace `cell`)
      and populate `node_position` from it instead of / in addition to the
      live fix.

## Notes / context

- `sdkconfig` is gitignored; config lives in tracked `sdkconfig.defaults`.
  After removing the debug flags there, delete `sdkconfig` (or
  `idf.py fullclean`) so it regenerates from defaults for the production build.
- Live GPS *during* PPP is deferred: the CMUX approach is preserved on
  `feature/cellular-x1-cmux` for post-deployment. CMUX is unstable on this
  SIM7600 firmware (state-machine restarts + Rx Breaks), so v1 uses the
  time-boxed pre-PPP fix.
- `scripts/pin_probe/` is a throwaway diagnostic sketch (untracked); not part
  of the firmware build.

See commit `039f460` for the full set of fixes in this branch.
