#!/usr/bin/env python3
"""SIGNATURE.sha256 검증 스크립트."""
from __future__ import annotations
import hashlib, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SIG = ROOT / "SIGNATURE.sha256"

def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()

errors = []
count = 0
for line in SIG.read_text().splitlines():
    line = line.strip()
    if not line:
        continue
    expected, rel = line.split("  ", 1)
    p = ROOT / rel
    count += 1
    if not p.exists():
        errors.append(f"MISSING: {rel}")
    else:
        actual = sha256_file(p)
        if actual != expected:
            errors.append(f"MISMATCH: {rel}")

if errors:
    for e in errors:
        print(e)
    sys.exit(1)
print(f"verify_signature: OK ({count} files)")
