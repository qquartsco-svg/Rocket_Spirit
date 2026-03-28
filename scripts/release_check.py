#!/usr/bin/env python3
"""릴리즈 전 전체 검증 파이프라인."""
from __future__ import annotations
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def run(cmd):
    r = subprocess.run(cmd, cwd=ROOT)
    if r.returncode != 0:
        sys.exit(r.returncode)

print("=== [1/3] pytest ===")
run([sys.executable, "-m", "pytest", "tests/", "-q", "--tb=short"])
print("=== [2/3] verify_signature ===")
run([sys.executable, "scripts/verify_signature.py"])
print("=== [3/3] cleanup ===")
run([sys.executable, "scripts/cleanup_generated.py"])
print("\nrelease_check: OK")
