"""Bounded local manipulation attempt journal; NOT physical execution evidence.

SQLite transactions precede command handoff. FULL synchronization depends on
the OS/storage honoring flushes; missing, replaced or damaged journals fail
closed. No database is auto-created on restart, and no attempt is ever deleted.
"""
from __future__ import annotations

import argparse
import json
import os
import sqlite3
import uuid
from pathlib import Path

from .fleet import robot_id_valid
from .manipulator_wire import MAX_OPERATIONS, identifier, validate_operation
from .wire_codec import decode_frame, encode_frame

MAX_DATABASE_BYTES = 32 * 1024 * 1024
EXECUTE_INTENT, ACCEPTED_ACK, STOP_REQUESTED, STOP_ACK = 1, 2, 4, 8
SCHEMA = {
    "metadata": "CREATE TABLE metadata (id INTEGER PRIMARY KEY CHECK(id=1), version INTEGER NOT NULL, journal_id TEXT NOT NULL, fleet TEXT NOT NULL)",
    "attempts": "CREATE TABLE attempts (work_id TEXT PRIMARY KEY, robot_id TEXT NOT NULL, session_id TEXT NOT NULL, command_id TEXT NOT NULL, operation TEXT NOT NULL, flags INTEGER NOT NULL, reviewer TEXT, evidence TEXT, UNIQUE(robot_id, session_id, command_id))",
}


class JournalError(ValueError):
    """No new manipulation may be dispatched using this failed operation."""


def _text(value):
    return type(value) is str and bool(value.strip()) and len(value) <= 1024 and "\x00" not in value


class OperationJournal:
    def __init__(self, connection, path, journal_id, fleet, *, writable):
        self._db, self.path, self.journal_id = connection, path, journal_id
        self.fleet, self._writable, self._broken = tuple(fleet), writable, False

    @classmethod
    def create(cls, path, *, robot_ids):
        if (type(robot_ids) not in (list, tuple) or not 1 <= len(robot_ids) <= 32
                or any(not robot_id_valid(r) for r in robot_ids) or len(set(robot_ids)) != len(robot_ids)):
            raise JournalError("Explicit unique fleet required")
        path = Path(path).absolute()
        # An interrupted creation leaves an invalid file, never a silently fresh DB.
        fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        os.close(fd)
        db = None
        journal_id = uuid.uuid4().hex
        try:
            db = sqlite3.connect(path, timeout=.2, isolation_level=None)
            db.execute("PRAGMA synchronous=FULL")
            db.execute("PRAGMA journal_mode=DELETE")
            db.execute("BEGIN IMMEDIATE")
            for sql in SCHEMA.values():
                db.execute(sql)
            db.execute("INSERT INTO metadata VALUES (1, 1, ?, ?)", (journal_id, json.dumps(sorted(robot_ids))))
            db.execute("COMMIT")
        finally:
            if db is not None:
                db.close()
        return cls.open(path, expected_id=journal_id)

    @classmethod
    def open(cls, path, *, expected_id):
        if not identifier(expected_id):
            raise JournalError("Pin the previously created journal ID; never discover a new ID to bypass recovery")
        return cls._open(path, expected_id=expected_id, writable=True)

    @classmethod
    def inspect_file(cls, path):
        with cls._open(path, expected_id=None, writable=False) as journal:
            return journal.snapshot()

    @classmethod
    def _open(cls, path, *, expected_id, writable):
        path = Path(path).absolute()
        db = None
        try:
            if path.is_symlink() or not path.is_file() or not 0 < path.stat().st_size <= MAX_DATABASE_BYTES:
                raise JournalError("Journal missing, empty, symlinked or oversized; do not recreate automatically")
            db = sqlite3.connect(path.as_uri() + ("?mode=rw" if writable else "?mode=ro"),
                                 uri=True, timeout=.2, isolation_level=None)
            db.row_factory = sqlite3.Row
            actual = {r["name"]: r["sql"] for r in db.execute("SELECT name, sql FROM sqlite_master WHERE sql IS NOT NULL")}
            if actual != SCHEMA or db.execute("PRAGMA integrity_check").fetchall()[0][0] != "ok":
                raise JournalError("Unrecognized or damaged journal schema")
            rows = db.execute("SELECT * FROM metadata").fetchall()
            if len(rows) != 1 or rows[0]["version"] != 1 or not identifier(rows[0]["journal_id"]):
                raise JournalError("Invalid journal metadata")
            metadata = rows[0]
            fleet = json.loads(metadata["fleet"])
            if (type(fleet) is not list or not 1 <= len(fleet) <= 32
                    or any(not robot_id_valid(r) for r in fleet) or sorted(set(fleet)) != fleet
                    or expected_id is not None and metadata["journal_id"] != expected_id):
                raise JournalError("Wrong journal identity or fleet")
            journal = cls(db, path, metadata["journal_id"], fleet, writable=writable)
            journal._validate_rows()
            if writable:
                db.execute("PRAGMA synchronous=FULL")
                if db.execute("PRAGMA journal_mode=DELETE").fetchone()[0] != "delete":
                    raise JournalError("Journal requires rollback journal mode")
            return journal
        except (OSError, sqlite3.Error, ValueError, TypeError, KeyError) as exc:
            if db is not None:
                db.close()
            raise JournalError(f"Cannot open trusted journal: {exc}") from exc

    def _validate_rows(self):
        rows = self._db.execute("SELECT * FROM attempts LIMIT ?", (MAX_OPERATIONS + 1,)).fetchall()
        if len(rows) > MAX_OPERATIONS:
            raise JournalError("Journal capacity exceeded")
        unresolved_robots = set()
        for row in rows:
            if type(row["operation"]) is not str:
                raise JournalError("Operation payload must be UTF-8 JSON text")
            operation = validate_operation(decode_frame(row["operation"].encode("utf-8") + b"\n"))
            flags = row["flags"]
            if (not identifier(row["work_id"]) or row["robot_id"] not in self.fleet
                    or not identifier(row["session_id"]) or row["command_id"] != operation["command_id"]
                    or type(flags) is not int or not 0 <= flags <= 15
                    or flags & ACCEPTED_ACK and not flags & EXECUTE_INTENT
                    or flags & STOP_ACK and not flags & STOP_REQUESTED):
                raise JournalError("Invalid attempt evidence")
            if row["reviewer"] is None:
                if row["evidence"] is not None or row["robot_id"] in unresolved_robots:
                    raise JournalError("Invalid unresolved operation set")
                unresolved_robots.add(row["robot_id"])
            elif not _text(row["reviewer"]) or not _text(row["evidence"]):
                raise JournalError("Invalid operator review")
        return rows

    def _transaction(self, action):
        if not self._writable or self._broken:
            raise JournalError("Journal is read-only, closed or faulted")
        try:
            self._db.execute("BEGIN IMMEDIATE")
            self._validate_rows()
            result = action()
            self._db.execute("COMMIT")
            return result
        except (sqlite3.Error, ValueError, TypeError, OSError) as exc:
            try:
                self._db.execute("ROLLBACK")
            except sqlite3.Error:
                pass
            # Contention or uncertain commit must never permit dispatch.
            self._broken = True
            raise JournalError(f"Journal transaction refused; reopen and inspect: {exc}") from exc

    def reserve(self, *, work_id, robot_id, session_id, operation):
        """Persist BEFORE hello; work_id is stable across session/robot changes."""
        operation = validate_operation(operation)
        if not identifier(work_id) or robot_id not in self.fleet or not identifier(session_id):
            raise JournalError("Stable work identity and pinned fleet required")
        encoded = encode_frame(operation).decode("utf-8").rstrip("\n")

        def action():
            if self._db.execute("SELECT 1 FROM attempts WHERE robot_id=? AND reviewer IS NULL", (robot_id,)).fetchone():
                raise JournalError("Robot has an unresolved attempt, even if the new session/work ID differs")
            if self._db.execute("SELECT count(*) FROM attempts").fetchone()[0] >= MAX_OPERATIONS:
                raise JournalError("History full; never evict replay protection")
            self._db.execute("INSERT INTO attempts VALUES (?, ?, ?, ?, ?, 0, NULL, NULL)",
                (work_id, robot_id, session_id, operation["command_id"], encoded))
        self._transaction(action)

    def mark(self, work_id, *, robot_id, session_id, flag):
        if flag not in (EXECUTE_INTENT, ACCEPTED_ACK, STOP_REQUESTED, STOP_ACK) or type(flag) is not int:
            raise JournalError("Unknown journal event")

        def action():
            row = self._db.execute("SELECT * FROM attempts WHERE work_id=?", (work_id,)).fetchone()
            if (row is None or row["robot_id"] != robot_id or row["session_id"] != session_id
                    or row["reviewer"] is not None):
                raise JournalError("Attempt ownership changed or quarantined")
            flags = row["flags"]
            if (flag == EXECUTE_INTENT and flags != 0
                    or flag == ACCEPTED_ACK and (not flags & EXECUTE_INTENT or flags & STOP_REQUESTED)
                    or flag == STOP_ACK and not flags & STOP_REQUESTED):
                raise JournalError("Invalid event ordering; execute is never replayed")
            self._db.execute("UPDATE attempts SET flags=? WHERE work_id=?", (flags | flag, work_id))
        self._transaction(action)

    def quarantine(self, work_id, *, reviewed_by, evidence, operator_confirmed_stopped):
        """Operator-declared reconciliation, never success, retry, or device stop.

        Stop/disable devices and the old controlling process BEFORE using this.
        Already handed-off bytes cannot be recalled by editing a local journal.
        """
        if operator_confirmed_stopped is not True or not _text(reviewed_by) or not _text(evidence):
            raise JournalError("Explicit stopped-device/process confirmation and written operator evidence required")

        def action():
            changed = self._db.execute("UPDATE attempts SET reviewer=?, evidence=? WHERE work_id=? AND reviewer IS NULL",
                                       (reviewed_by, evidence, work_id)).rowcount
            if changed != 1:
                raise JournalError("No unresolved attempt with this work ID")
        self._transaction(action)

    def snapshot(self):
        if self._broken:
            raise JournalError("Journal closed or faulted; reopen for inspection")
        try:
            rows = self._validate_rows()
            return {"schema_version": 1, "journal_id": self.journal_id, "robot_ids": list(self.fleet),
                "physical_success": None, "automatic_replay_enabled": False,
                "attempts": [{"work_id": r["work_id"], "robot_id": r["robot_id"], "session_id": r["session_id"],
                    "operation": json.loads(r["operation"]), "flags": r["flags"],
                    "state": "quarantined_never_retry" if r["reviewer"] is not None else "unresolved",
                    "reviewed_by": r["reviewer"], "evidence": r["evidence"], "physical_success": None}
                    for r in rows]}
        except (sqlite3.Error, ValueError, TypeError) as exc:
            self._broken = True
            raise JournalError(f"Invalid journal: {exc}") from exc

    def close(self):
        self._db.close()
        self._broken = True

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    inspect = sub.add_parser("inspect", help="Read-only, no recovery or dispatch")
    inspect.add_argument("path", type=Path)
    create = sub.add_parser("init", help="Explicit NEW local journal; never overwrite")
    create.add_argument("path", type=Path)
    create.add_argument("--robots", nargs="+", required=True)
    quarantine = sub.add_parser("quarantine", help="Keep attempted work permanently blocked; not a retry")
    quarantine.add_argument("path", type=Path)
    quarantine.add_argument("--journal-id", required=True)
    quarantine.add_argument("--work-id", required=True)
    quarantine.add_argument("--reviewed-by", required=True)
    quarantine.add_argument("--evidence", required=True)
    quarantine.add_argument("--confirm-devices-and-old-process-stopped", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.command == "inspect":
            result = OperationJournal.inspect_file(args.path)
        elif args.command == "init":
            with OperationJournal.create(args.path, robot_ids=args.robots) as journal:
                result = journal.snapshot()
        else:
            with OperationJournal.open(args.path, expected_id=args.journal_id) as journal:
                journal.quarantine(args.work_id, reviewed_by=args.reviewed_by, evidence=args.evidence,
                    operator_confirmed_stopped=args.confirm_devices_and_old_process_stopped)
                result = journal.snapshot()
        print(json.dumps(result, ensure_ascii=True, indent=2))
        return 0
    except (OSError, sqlite3.Error, ValueError) as exc:
        parser.exit(1, f"operation-journal: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
