# Documentation Index — Rustacean RoboRescue

This folder contains operational and developer-facing documentation for the project. Use this index to quickly navigate the most important guides.

## Table of contents

- [DEV_QUICKSTART.md](DEV_QUICKSTART.md) — Quick developer setup (Nix / Docker / Just)
- [NIX.md](NIX.md) — Nix flake configuration and troubleshooting
- [PACKAGES.md](PACKAGES.md) — Workspace package inventory and entry points
- [ARCHITECTURE.md](ARCHITECTURE.md) — System architecture, node map, topics and data flows
- [OPERATION.md](OPERATION.md) — Build, run, deploy, and troubleshooting
- [ROBOC_IMPLEMENTATION_PLAN.md](ROBOC_IMPLEMENTATION_PLAN.md) — Roadmap and plans
- [RTP_FALLBACK.md](RTP_FALLBACK.md) — RTP fallback and GStreamer notes
- [RULES.md](RULES.md) — Competition rules (text)
- [RoboCupRescue-Rules-2026B.txt](RoboCupRescue-Rules-2026B.txt) — Official rules text used in this repo

## Conventions

- All docs use UTF-8 and Markdown (CommonMark). Keep short paragraphs and a clear action-first tone.
- Use present-tense instructions and provide example commands in bash blocks.
- For any change that affects developer workflows (Justfile, CI, devcontainer), update both `DEV_QUICKSTART.md` and the relevant package README or operation guide.

## How you can help

- Spellcheck and run link-checks before merging.
- Add short usage examples to package-level READMEs (for example `main_ws/src/audio_bridge/README.md`).
- If you add CLI flags, update `OPERATION.md` and the example launch snippets.
