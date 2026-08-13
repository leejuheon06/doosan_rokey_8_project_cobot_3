"""SQLite history for Speedy Double UI-launched executions."""

from __future__ import annotations

import json
import sqlite3
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path


DB_PATH = Path(__file__).parent / "palletizing_runs.db"
SCHEMA = """
CREATE TABLE IF NOT EXISTS executions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_uid TEXT NOT NULL UNIQUE,
    created_at TEXT NOT NULL,
    status TEXT NOT NULL,
    exit_code INTEGER,
    settings_json TEXT NOT NULL,
    result_json TEXT NOT NULL
)
"""


@contextmanager
def get_conn(db_path: Path = DB_PATH):
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        yield conn
        conn.commit()
    finally:
        conn.close()


def init_db(db_path: Path = DB_PATH) -> None:
    with get_conn(db_path) as conn:
        conn.execute(SCHEMA)


def save_execution(result: dict, db_path: Path = DB_PATH) -> int:
    run_uid = str(result["run_uid"])
    settings = result.get("settings", {})
    with get_conn(db_path) as conn:
        conn.execute(
            """INSERT OR IGNORE INTO executions
               (run_uid, created_at, status, exit_code, settings_json, result_json)
               VALUES (?, ?, ?, ?, ?, ?)""",
            (
                run_uid,
                datetime.now().isoformat(timespec="seconds"),
                str(result.get("status", "unknown")),
                result.get("exit_code"),
                json.dumps(settings, ensure_ascii=False),
                json.dumps(result, ensure_ascii=False),
            ),
        )
        row = conn.execute("SELECT id FROM executions WHERE run_uid = ?", (run_uid,)).fetchone()
        return int(row["id"])


def _decoded(row) -> dict:
    item = dict(row)
    item["settings"] = json.loads(item.pop("settings_json"))
    item["result"] = json.loads(item.pop("result_json"))
    return item


def fetch_executions(limit: int = 100, db_path: Path = DB_PATH) -> list[dict]:
    with get_conn(db_path) as conn:
        rows = conn.execute(
            "SELECT * FROM executions ORDER BY id DESC LIMIT ?", (max(1, int(limit)),)
        ).fetchall()
        return [_decoded(row) for row in rows]


def fetch_execution(run_id: int, db_path: Path = DB_PATH) -> dict | None:
    with get_conn(db_path) as conn:
        row = conn.execute("SELECT * FROM executions WHERE id = ?", (run_id,)).fetchone()
        return _decoded(row) if row else None


def delete_execution(run_id: int, db_path: Path = DB_PATH) -> bool:
    with get_conn(db_path) as conn:
        return conn.execute("DELETE FROM executions WHERE id = ?", (run_id,)).rowcount > 0


def clear_executions(db_path: Path = DB_PATH) -> int:
    with get_conn(db_path) as conn:
        return conn.execute("DELETE FROM executions").rowcount
