"""Preservation and overwrite guards for the connected documentation mirror."""
import json

import pytest

from scripts.sync_engineering_guide import DOCUMENT_ID, GUIDE, MANIFEST, render, synchronize


def paragraph(text, kind="NORMAL_TEXT", style=None):
    return {"paragraph": {"paragraphStyle": {"namedStyleType": kind},
        "elements": [{"textRun": {"content": text + "\n", "textStyle": style or {}}}]}}


def document():
    return {"documentId": DOCUMENT_ID, "revisionId": "test-revision", "title": "Guide",
        "tabs": [{"tabId": "t.0", "title": "First tab", "body": {"content": [
            paragraph("First tab", "TITLE"), paragraph("Section", "HEADING_1"),
            paragraph("Keep <angle> & [brackets] and a_b.")]}},
            {"tabId": "t.second", "title": "Second tab", "body": {"content": [
                paragraph("python demo.py --speed 3", style={"weightedFontFamily": {"fontFamily": "Consolas"}}),
                {"table": {"columns": 2, "tableRows": [
                    {"tableCells": [{"content": [paragraph("Name")]}, {"content": [paragraph("Value")]}]},
                    {"tableCells": [{"content": [paragraph("A|B")]}, {"content": [paragraph("3")]}]}
                ]}}]}}]}


def test_complete_tabs_tables_commands_and_text_survive():
    payload, manifest = render(document())
    text = payload.decode()
    assert "## First tab" in text and "## Second tab" in text
    assert "### Section" in text
    assert "Keep &lt;angle&gt; &amp; \\[brackets\\] and a\\_b." in text
    assert "```text\npython demo.py --speed 3\n```" in text
    assert "| A&#124;B | 3 |" in text
    assert manifest["totals"] == dict(tabs=2, paragraphs=8, tables=1, code_blocks=1)


def test_local_edits_and_missing_tabs_cannot_be_overwritten(tmp_path):
    synchronize(document(), tmp_path)
    original = (tmp_path / GUIDE).read_bytes()
    (tmp_path / GUIDE).write_bytes(original + b"\nHuman edit\n")
    with pytest.raises(ValueError, match="Local Markdown diverged"):
        synchronize(document(), tmp_path)
    assert (tmp_path / GUIDE).read_bytes().endswith(b"Human edit\n")
    (tmp_path / GUIDE).write_bytes(original)
    partial = document()
    partial["tabs"].pop()
    with pytest.raises(ValueError, match="Missing source tabs"):
        synchronize(partial, tmp_path)
    assert (tmp_path / GUIDE).read_bytes() == original


def test_check_is_read_only_and_revision_changes_are_visible(tmp_path):
    synchronize(document(), tmp_path)
    before = (tmp_path / MANIFEST).read_bytes()
    synchronize(document(), tmp_path, check=True)
    edited = document()
    edited["revisionId"] = "new-revision"
    with pytest.raises(ValueError, match="does not match"):
        synchronize(edited, tmp_path, check=True)
    assert (tmp_path / MANIFEST).read_bytes() == before
    synchronize(edited, tmp_path)
    assert json.loads((tmp_path / MANIFEST).read_text())["source_revision_id"] == "new-revision"


@pytest.mark.parametrize("unsupported", ["inlineObjects", "footnotes", "headers", "childTabs"])
def test_unsupported_features_abort_before_any_write(tmp_path, unsupported):
    source = document()
    source["tabs"][0][unsupported] = {"new": {"content": "must not vanish"}}
    with pytest.raises(ValueError, match="needs explicit conversion"):
        synchronize(source, tmp_path)
    assert not (tmp_path / GUIDE).exists()


def test_wrong_document_and_unsupported_native_elements_are_rejected():
    source = document()
    source["documentId"] = "another-document"
    with pytest.raises(ValueError, match="bound to"):
        render(source)
    source = document()
    source["tabs"][0]["body"]["content"].append({"paragraph": {"elements": [{"inlineObjectElement": {}}]}})
    with pytest.raises(ValueError, match="Unsupported native paragraph"):
        render(source)


def test_links_with_parentheses_and_literal_code_backticks():
    source = document()
    source["tabs"][0]["body"]["content"] = [
        paragraph("A link", style={"link": {"url": "https://example.com/a_(b)"}}),
        paragraph("`literal`", style={"weightedFontFamily": {"fontFamily": "Consolas"}})]
    text = render(source)[0].decode()
    assert "[A link](<https://example.com/a_(b)>)" in text
    assert "```text\n`literal`\n```" in text


def test_literal_numbering_and_full_commit_url_labels_are_preserved():
    source = document()
    commit = "a" * 40
    source["tabs"][0]["body"]["content"] = [
        paragraph("12. Keep the original number."),
        paragraph(f"Commit {commit}; PR https://github.com/example/repo/pull/10.")]
    text = render(source)[0].decode()
    assert "12\\. Keep the original number." in text
    assert f"`{commit}`" in text
    assert "[`https://github.com/example/repo/pull/10`](<https://github.com/example/repo/pull/10>)." in text
