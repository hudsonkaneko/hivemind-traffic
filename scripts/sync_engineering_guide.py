"""Render the connected Google guide's full native read into a GitHub mirror.

Authentication and native edits belong to the Google Drive connector. This
offline renderer accepts its get_document structuredContent, never credentials.
It refuses unsupported content and local divergence instead of losing either.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import json
from pathlib import Path
import re
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
DOCUMENT_ID = "1qeYucQnP4nznmLNJrlUz1hCIcY7KOigRPuhBVH-Zo0c"
DOCUMENT_URL = f"https://docs.google.com/document/d/{DOCUMENT_ID}/edit"
GUIDE = Path("documentation/engineering-guide.md")
MANIFEST = Path("documentation/engineering-guide.sync.json")
MONOSPACE = {"consolas", "courier new", "courier", "monospace"}


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def escaped(text: str) -> str:
    text = html.escape(text, quote=False)
    return re.sub(r"([\\`*_{}\[\]#!|])", r"\\\1", text)


def target(url: str) -> str:
    if urlsplit(url).scheme not in {"http", "https", "mailto"}:
        raise ValueError(f"Unsupported link scheme: {urlsplit(url).scheme}")
    return url.replace(" ", "%20").replace("<", "%3C").replace(">", "%3E")


def prose(text: str) -> str:
    """Keep full commit IDs and URL labels visible under GitHub auto-linking."""
    chunks = re.split(r"(https?://[^\s<>]+|\b[0-9a-f]{40}\b)", text)
    output = []
    for chunk in chunks:
        if chunk.startswith(("https://", "http://")):
            url = chunk.rstrip(".,;:")
            while url.endswith(")") and url.count(")") > url.count("("):
                url = url[:-1]
            label = "`" + url + "`" if "`" not in url else escaped(url)
            output.append("[" + label + "](<" + target(url) + ">)" + escaped(chunk[len(url):]))
        elif re.fullmatch(r"[0-9a-f]{40}", chunk):
            output.append("`" + chunk + "`")
        else:
            output.append(escaped(chunk))
    return "".join(output)


def anchor(identifier: str) -> str:
    if not re.fullmatch(r"[a-zA-Z0-9_.-]+", identifier):
        raise ValueError("Unexpected native tab/heading identifier")
    return "docs-" + identifier.replace(".", "-")


def runs(paragraph: dict) -> list[dict]:
    result = []
    if paragraph.get("bullet"):
        raise ValueError("Native list needs an explicit conversion; no content written")
    for element in paragraph.get("elements", []):
        if set(element) - {"startIndex", "endIndex", "textRun"} or "textRun" not in element:
            raise ValueError("Unsupported native paragraph element; no content written")
        run = element["textRun"]
        if set(run) - {"content", "textStyle"}:
            raise ValueError("Suggested or unsupported text change needs review")
        result.append(run)
    return result


def code_paragraph(paragraph: dict) -> bool:
    nonempty = [r for r in runs(paragraph) if r["content"].strip()]
    return bool(nonempty) and all(
        r.get("textStyle", {}).get("weightedFontFamily", {}).get("fontFamily", "").lower()
        in MONOSPACE for r in nonempty
    )


def inline(paragraph: dict) -> str:
    result = []
    for run in runs(paragraph):
        raw = run["content"].rstrip("\n")
        if not raw:
            continue
        style = run.get("textStyle", {})
        if any(k.startswith("suggested") and v for k, v in style.items()):
            raise ValueError("Suggested formatting needs review")
        leading = raw[:len(raw) - len(raw.lstrip())]
        trailing = raw[len(raw.rstrip()):]
        core = raw.strip()
        if not core:
            result.append(raw)
            continue
        mono = style.get("weightedFontFamily", {}).get("fontFamily", "").lower() in MONOSPACE
        if mono and "\n" not in core:
            fence = "`" * (max([len(x) for x in re.findall(r"`+", core)] or [0]) + 1)
            padding = " " if core.startswith("`") or core.endswith("`") else ""
            value = fence + padding + core + padding + fence
        else:
            value = (escaped(core) if style.get("link") else prose(core)).replace("\n", "<br>\n")
            if style.get("bold"):
                value = "**" + value + "**"
            if style.get("italic"):
                value = "*" + value + "*"
            if style.get("strikethrough"):
                value = "~~" + value + "~~"
        if style.get("link"):
            link = style["link"]
            if "url" not in link:
                raise ValueError("Native internal link needs explicit mapping")
            if core == link["url"] and "`" not in core:
                value = "`" + core + "`"
            value = "[" + value + "](<" + target(link["url"]) + ">)"
        result.append(leading + value + trailing)
    return "".join(result).strip()


def table_markdown(table: dict) -> str:
    rows = []
    for row in table["tableRows"]:
        cells = []
        for cell in row["tableCells"]:
            style = cell.get("tableCellStyle", {})
            if style.get("rowSpan", 1) != 1 or style.get("columnSpan", 1) != 1:
                raise ValueError("Merged table needs explicit conversion")
            paragraphs = []
            for element in cell["content"]:
                if "paragraph" not in element:
                    raise ValueError("Nested table content needs explicit conversion")
                paragraphs.append(inline(element["paragraph"]).replace("\n", "<br>"))
            cells.append("<br>".join(p for p in paragraphs if p).replace("\\|", "&#124;").replace("|", "&#124;"))
        rows.append(cells)
    if not rows or any(len(row) != table["columns"] for row in rows):
        raise ValueError("Irregular native table")
    lines = ["| " + " | ".join(row) + " |" for row in rows]
    lines.insert(1, "| " + " | ".join(["---"] * table["columns"]) + " |")
    return "\n".join(lines)


def render(document: dict) -> tuple[bytes, dict]:
    if document.get("documentId") != DOCUMENT_ID:
        raise ValueError("This renderer is bound to the project's engineering guide")
    tabs = document.get("tabs")
    if not tabs or not document.get("revisionId"):
        raise ValueError("A full native read with all tabs and revisionId is required")
    if len({tab["tabId"] for tab in tabs}) != len(tabs):
        raise ValueError("Duplicate tab IDs")
    output = [f"# {escaped(document['title'])}",
        f"[Google document]({DOCUMENT_URL}) · [Update workflow](learning-guide-sync.md#chat-managed-markdown-mirror)",
        "This is the complete GitHub-readable copy of the Google guide. Milestone "
        "entries retain their original dates and claims; they are historical evidence, "
        "not a substitute for the [current implementation notes](README.md). "
        "Updates made through the project chat refresh this file after native readback. "
        "This is chat-managed synchronization, not a background Google Docs watcher.",
        "## Contents"]
    output.append("\n".join(f"- [{escaped(t['title'])}](#{anchor(t['tabId'])})" for t in tabs))
    inventory = []
    total_tables = total_paragraphs = code_blocks = 0
    for tab in tabs:
        for key in ("inlineObjects", "positionedObjects", "headers", "footers", "footnotes", "childTabs"):
            if tab.get(key):
                raise ValueError(f"{tab['title']}: {key} needs explicit conversion")
        if tab.get("body") is None:
            raise ValueError(f"Missing full body for {tab['title']}")
        output.extend([f'<a id="{anchor(tab["tabId"])}"></a>', "## " + escaped(tab["title"])])
        paragraphs = table_count = 0
        for element in tab["body"]["content"]:
            if "sectionBreak" in element:
                continue
            if "table" in element:
                output.append(table_markdown(element["table"]))
                table_count += 1
                paragraphs += sum(len(c["content"]) for r in element["table"]["tableRows"] for c in r["tableCells"])
                continue
            if "paragraph" not in element:
                raise ValueError(f"Unsupported body content in {tab['title']}")
            paragraph = element["paragraph"]
            paragraphs += 1
            raw = "".join(r["content"] for r in runs(paragraph)).rstrip("\n")
            if not raw.strip():
                continue
            style = paragraph.get("paragraphStyle", {})
            kind = style.get("namedStyleType", "NORMAL_TEXT")
            if kind == "TITLE" and raw.strip() == tab["title"]:
                continue  # Its text is already the tab section heading.
            if style.get("headingId"):
                output.append(f'<a id="{anchor(style["headingId"])}"></a>')
            if code_paragraph(paragraph):
                fence = "`" * max(3, max([len(x) for x in re.findall(r"`+", raw)] or [0]) + 1)
                output.append(fence + "text\n" + raw + "\n" + fence)
                code_blocks += 1
            elif kind.startswith("HEADING_") or kind in {"TITLE", "SUBTITLE"}:
                level = min(6, int(kind[-1]) + 2) if kind.startswith("HEADING_") else 3
                output.append("#" * level + " " + inline(paragraph))
            else:
                text = inline(paragraph)
                # These are literal numbered paragraphs, not native Docs lists.
                text = re.sub(r"^(\d+)([.)])(?=\s)", r"\1\\\2", text)
                text = re.sub(r"^([-+>])(?=\s)", r"\\\1", text)
                output.append(text)
        inventory.append(dict(tab_id=tab["tabId"], title=tab["title"],
            parent_tab_id=tab.get("parentTabId"), paragraphs=paragraphs, tables=table_count,
            source_body_sha256=sha(json.dumps(tab["body"], sort_keys=True, ensure_ascii=False).encode())))
        total_paragraphs += paragraphs
        total_tables += table_count
    payload = ("\n\n".join(output) + "\n").encode("utf-8")
    manifest = dict(schema_version=1, document_id=DOCUMENT_ID, document_url=DOCUMENT_URL,
        source_revision_id=document["revisionId"], markdown_path=GUIDE.as_posix(),
        markdown_sha256=sha(payload), sync_mode="chat_managed_after_verified_google_edit",
        renderer="scripts/sync_engineering_guide.py", tabs=inventory,
        totals=dict(tabs=len(tabs), paragraphs=total_paragraphs, tables=total_tables, code_blocks=code_blocks),
        limitations=["Typography, page layout, color and comment threads are not mirrored.",
            "No background polling or automatic sync for edits made outside the chat.",
            "Unsupported native elements stop conversion rather than silently disappearing."])
    return payload, manifest


def synchronize(document: dict, root: Path, *, check: bool = False) -> dict:
    payload, manifest = render(document)
    guide, record = root / GUIDE, root / MANIFEST
    if guide.exists() != record.exists():
        raise ValueError("Mirror/manifest pair incomplete; reconcile before refreshing")
    if record.exists():
        previous = json.loads(record.read_text(encoding="utf-8"))
        if sha(guide.read_bytes()) != previous["markdown_sha256"]:
            raise ValueError("Local Markdown diverged; reconcile with Google before refreshing")
        missing = {t["tab_id"] for t in previous["tabs"]} - {t["tab_id"] for t in manifest["tabs"]}
        if missing:
            raise ValueError("Missing source tabs; confirm a complete read and preserve history")
    record_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode()
    if check:
        if not guide.exists() or guide.read_bytes() != payload or record.read_bytes() != record_bytes:
            raise ValueError("Mirror does not match this source revision")
    else:
        guide.parent.mkdir(parents=True, exist_ok=True)
        guide.write_bytes(payload)
        record.write_bytes(record_bytes)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True, help="Full get_document structuredContent JSON under outputs/")
    parser.add_argument("--check", action="store_true", help="Verify this source matches the mirror without writing")
    args = parser.parse_args()
    if not Path.cwd().resolve().is_relative_to(ROOT):
        parser.error("Run inside this repository")
    source = args.source.resolve()
    if not source.is_relative_to(ROOT / "outputs"):
        parser.error("Keep authenticated read snapshots under this repository's ignored outputs/")
    document = json.loads(source.read_text(encoding="utf-8"))
    document = document.get("structuredContent", document)
    try:
        manifest = synchronize(document, ROOT, check=args.check)
    except (ValueError, KeyError) as error:
        parser.exit(1, str(error) + "\n")
    print(json.dumps({"verified" if args.check else "updated": GUIDE.as_posix(), **manifest["totals"]}))


if __name__ == "__main__":
    main()
