#!/usr/bin/env python3
"""캐시 및 생성 파일 정리."""
from __future__ import annotations
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
removed = 0
for p in ROOT.rglob("__pycache__"):
    shutil.rmtree(p); removed += 1
for p in ROOT.rglob(".pytest_cache"):
    shutil.rmtree(p); removed += 1
for p in ROOT.rglob("*.pyc"):
    p.unlink(); removed += 1
for p in ROOT.rglob(".DS_Store"):
    p.unlink(); removed += 1
print(f"cleanup: {removed} items removed")
