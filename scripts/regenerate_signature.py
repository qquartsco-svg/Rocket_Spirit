#!/usr/bin/env python3
"""SIGNATURE.sha256 재생성 스크립트."""
from __future__ import annotations
import hashlib, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKIP_DIRS = {".git", ".pytest_cache", "__pycache__", "dist", "build", ".eggs"}
SKIP_FILES = {"SIGNATURE.sha256"}

def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()

entries = []
for path in sorted(ROOT.rglob("*")):
    if not path.is_file():
        continue
    if any(d in path.parts for d in SKIP_DIRS):
        continue
    if path.suffix == ".pyc" or path.name in SKIP_FILES:
        continue
    rel = path.relative_to(ROOT)
    entries.append(f"{sha256_file(path)}  {rel}")

sig_path = ROOT / "SIGNATURE.sha256"
sig_path.write_text("\n".join(entries) + "\n")
print(f"Wrote {len(entries)} entries to {sig_path}")
