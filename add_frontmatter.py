"""Sync markdown files from the workspace into Hexo posts with Git-based dates."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable


DEFAULT_CONFIG = {
    "include_roots": [],
    "exclude_prefixes": [
        "Blogs/",
        ".git/",
        ".github/",
    ],
    "exclude_file_names": [],
    "category_aliases": {
        "Cpp": "C++",
        "node.js": "Node.js",
    },
    "output_subdir": "Blogs/source/_posts/synced",
}

HEADING_RE = re.compile(r"^#{1,6}\s+(.+?)\s*$")
TOKEN_RE = re.compile(r"[^a-zA-Z0-9]+")


@dataclass(frozen=True)
class SyncConfig:
    include_roots: list[str]
    exclude_prefixes: list[str]
    exclude_file_names: set[str]
    category_aliases: dict[str, str]
    output_subdir: str


def load_config(repo_root: Path, config_path: Path) -> SyncConfig:
    merged = dict(DEFAULT_CONFIG)
    if config_path.exists():
        data = json.loads(config_path.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("Sync config must be a JSON object")
        merged.update(data)

    include_roots = merged.get("include_roots") or []
    if not include_roots:
        include_roots = [
            p.name for p in repo_root.iterdir() if p.is_dir() and not p.name.startswith(".")
        ]

    return SyncConfig(
        include_roots=sorted(set(include_roots)),
        exclude_prefixes=[normalize_prefix(p) for p in merged.get("exclude_prefixes", [])],
        exclude_file_names=set(merged.get("exclude_file_names", [])),
        category_aliases=dict(merged.get("category_aliases", {})),
        output_subdir=str(merged.get("output_subdir", DEFAULT_CONFIG["output_subdir"])),
    )


def normalize_prefix(value: str) -> str:
    value = value.replace("\\", "/").lstrip("./")
    if value and not value.endswith("/"):
        value += "/"
    return value


def parse_git_date_listing(raw: str) -> dict[str, str]:
    mapping: dict[str, str] = {}
    current_date: str | None = None
    for line in raw.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("__COMMIT__"):
            current_date = line.removeprefix("__COMMIT__").strip()
            continue
        if current_date and line.endswith(".md") and line not in mapping:
            mapping[line] = current_date
    return mapping


def build_git_date_cache(repo_root: Path) -> tuple[dict[str, str], dict[str, str]]:
    updated_raw = run_git(
        repo_root,
        [
            "log",
            "--format=__COMMIT__%aI",
            "--name-only",
            "--",
            "*.md",
        ],
    )
    created_raw = run_git(
        repo_root,
        [
            "log",
            "--diff-filter=A",
            "--format=__COMMIT__%aI",
            "--name-only",
            "--",
            "*.md",
        ],
    )
    updated_map = parse_git_date_listing(updated_raw)
    created_map = parse_git_date_listing(created_raw)
    return created_map, updated_map


def run_git(repo_root: Path, args: list[str]) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return ""
    return result.stdout.strip()


def to_hexo_time(iso_time: str | None, fallback_epoch: float) -> str:
    if iso_time:
        try:
            dt = datetime.fromisoformat(iso_time.replace("Z", "+00:00"))
            return dt.astimezone(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
        except ValueError:
            pass
    return datetime.fromtimestamp(fallback_epoch, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")


def strip_frontmatter(content: str) -> str:
    if not content.startswith("---\n"):
        return content
    end = content.find("\n---\n", 4)
    if end == -1:
        return content
    return content[end + 5 :].lstrip("\n")


def infer_title(file_path: Path, body: str) -> str:
    for line in body.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        match = HEADING_RE.match(stripped)
        if match:
            return match.group(1).strip()
        break
    title = file_path.stem.replace("_", " ").replace("-", " ").strip()
    return title or file_path.stem


def normalize_tag(value: str) -> str:
    lowered = value.lower()
    lowered = TOKEN_RE.sub("-", lowered)
    lowered = lowered.strip("-")
    return lowered


def get_categories(rel_path: Path, aliases: dict[str, str]) -> list[str]:
    parts = list(rel_path.parent.parts)
    if not parts:
        return ["Uncategorized"]
    return [aliases.get(p, p) for p in parts]


def get_tags(rel_path: Path) -> list[str]:
    tokens: list[str] = []
    for part in rel_path.parent.parts:
        tag = normalize_tag(part)
        if tag:
            tokens.append(tag)
    stem_tag = normalize_tag(rel_path.stem)
    if stem_tag:
        tokens.append(stem_tag)
    unique = sorted(set(tokens))
    return unique or ["notes"]


def yaml_quote(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def build_frontmatter(
    title: str,
    created: str,
    updated: str,
    categories: Iterable[str],
    tags: Iterable[str],
    source_path: str,
    source_hash: str,
) -> str:
    category_lines = "\n".join(f"  - {yaml_quote(c)}" for c in categories)
    tag_lines = "\n".join(f"  - {yaml_quote(t)}" for t in tags)
    return (
        "---\n"
        f"title: {yaml_quote(title)}\n"
        f"date: {created}\n"
        f"updated: {updated}\n"
        "categories:\n"
        f"{category_lines}\n"
        "tags:\n"
        f"{tag_lines}\n"
        f"source_path: {yaml_quote(source_path)}\n"
        f"source_hash: {source_hash}\n"
        "---\n\n"
    )


def should_skip(rel_posix: str, file_name: str, config: SyncConfig) -> bool:
    if file_name in config.exclude_file_names:
        return True
    for prefix in config.exclude_prefixes:
        if rel_posix.startswith(prefix):
            return True
    return False


def iter_markdown_files(repo_root: Path, config: SyncConfig) -> list[Path]:
    files: list[Path] = []
    for include_root in config.include_roots:
        root = repo_root / include_root
        if not root.exists() or not root.is_dir():
            continue
        for path in root.rglob("*.md"):
            if not path.is_file():
                continue
            rel_posix = path.relative_to(repo_root).as_posix()
            if should_skip(rel_posix, path.name, config):
                continue
            files.append(path)
    files.sort()
    return files


def sync_markdown(repo_root: Path, config: SyncConfig, dry_run: bool, clean: bool) -> dict[str, int]:
    output_root = (repo_root / config.output_subdir).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    markdown_files = iter_markdown_files(repo_root, config)
    created_map, updated_map = build_git_date_cache(repo_root)
    expected_targets: set[Path] = set()

    stats = {
        "processed": 0,
        "written": 0,
        "unchanged": 0,
        "deleted": 0,
        "git_fallback": 0,
    }

    for source_file in markdown_files:
        stats["processed"] += 1
        rel_path = source_file.relative_to(repo_root)
        rel_posix = rel_path.as_posix()
        target_path = output_root / rel_path
        expected_targets.add(target_path)
        target_path.parent.mkdir(parents=True, exist_ok=True)

        raw = source_file.read_text(encoding="utf-8")
        body = strip_frontmatter(raw)
        title = infer_title(source_file, body)

        created_iso = created_map.get(rel_posix)
        updated_iso = updated_map.get(rel_posix)
        if not created_iso or not updated_iso:
            stats["git_fallback"] += 1

        created = to_hexo_time(created_iso, source_file.stat().st_mtime)
        updated = to_hexo_time(updated_iso, source_file.stat().st_mtime)
        categories = get_categories(rel_path, config.category_aliases)
        tags = get_tags(rel_path)
        source_hash = hashlib.sha1(body.encode("utf-8")).hexdigest()[:12]

        frontmatter = build_frontmatter(
            title=title,
            created=created,
            updated=updated,
            categories=categories,
            tags=tags,
            source_path=rel_posix,
            source_hash=source_hash,
        )
        merged = frontmatter + body

        if target_path.exists() and target_path.read_text(encoding="utf-8") == merged:
            stats["unchanged"] += 1
            continue

        stats["written"] += 1
        if not dry_run:
            target_path.write_text(merged, encoding="utf-8")

    if clean:
        for stale_file in output_root.rglob("*.md"):
            if stale_file not in expected_targets:
                stats["deleted"] += 1
                if not dry_run:
                    stale_file.unlink()

    return stats


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sync markdown files into Hexo posts")
    parser.add_argument(
        "--config",
        default=".blog-sync.json",
        help="Path to sync config file (default: .blog-sync.json)",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print actions without writing files")
    parser.add_argument("--clean", action="store_true", default=False, help="Delete stale synced files")
    parser.add_argument(
        "--no-clean",
        action="store_true",
        help="Do not delete stale synced files",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent
    config_path = (repo_root / args.config).resolve()
    config = load_config(repo_root, config_path)

    clean = args.clean and not args.no_clean
    stats = sync_markdown(repo_root, config, dry_run=args.dry_run, clean=clean)

    print("Markdown sync report")
    print(f"  processed: {stats['processed']}")
    print(f"  written: {stats['written']}")
    print(f"  unchanged: {stats['unchanged']}")
    print(f"  deleted: {stats['deleted']}")
    print(f"  git_fallback: {stats['git_fallback']}")
    print(f"  output_dir: {config.output_subdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
