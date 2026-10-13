"""CPU-only checks for the reviewed repo-local instruction bundle."""

import json
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SKILLS = ROOT / ".agents" / "skills"
NAMES = {
    "sumo-traci-traffic", "pettingzoo-multiagent-traffic", "train-hivemind-marl",
    "stream-traffic-openusd", "build-omniverse-traffic-viewer",
    "validate-av-isaac-lab", "run-reproducible-av-experiments",
    "prepare-simready-vehicle",
}


class ProjectSkillsTests(unittest.TestCase):
    def test_reviewed_inventory_and_provenance(self):
        manifest = json.loads((ROOT / "documentation/project-skills-manifest.json").read_text(encoding="utf-8"))
        actual = {p.relative_to(ROOT).as_posix() for p in SKILLS.rglob("*") if p.is_file()}
        declared = [item["path"] for item in manifest["files"]]
        self.assertEqual(set(declared), actual)
        self.assertEqual(len(declared), len(set(declared)))
        self.assertEqual(len(actual), 18)
        self.assertEqual({p.name for p in SKILLS.iterdir() if p.is_dir()}, NAMES)
        for item in manifest["files"]:
            self.assertRegex(item["source_sha256"], r"^[0-9a-f]{64}$")

    def test_skill_metadata_and_project_scope(self):
        for name in sorted(NAMES):
            with self.subTest(skill=name):
                text = (SKILLS / name / "SKILL.md").read_text(encoding="utf-8")
                match = re.match(r"\A---\n(.*?)\n---\n", text, re.S)
                self.assertIsNotNone(match)
                fields = dict(line.split(": ", 1) for line in match.group(1).splitlines())
                self.assertEqual(fields["name"], name)
                self.assertTrue(0 < len(fields["description"]) <= 1024)
                self.assertIn("## Highway Sim repository scope", text)
                self.assertIn("documentation/project-skills.md", text)

    def test_ui_metadata(self):
        for name in sorted(NAMES):
            with self.subTest(skill=name):
                text = (SKILLS / name / "agents/openai.yaml").read_text(encoding="utf-8")
                self.assertTrue(text.startswith("interface:\n"))
                fields = {key: json.loads(value) for key, value in
                          re.findall(r'^  (\w+): (".*")$', text, re.M)}
                self.assertTrue(fields["display_name"])
                self.assertTrue(25 <= len(fields["short_description"]) <= 64)
                self.assertIn("$" + name, fields["default_prompt"])

    def test_local_reference_closure(self):
        for path in SKILLS.rglob("*.md"):
            text = path.read_text(encoding="utf-8")
            for target in re.findall(r"\[[^\]]+\]\(([^)]+)\)", text):
                if re.match(r"https?://|#", target):
                    continue
                resolved = (path.parent / target.split("#", 1)[0]).resolve()
                self.assertTrue(resolved.is_relative_to(SKILLS.resolve()))
                self.assertTrue(resolved.is_file(), f"Missing reference: {path}: {target}")

    def test_no_machine_paths_credentials_or_binary_payloads(self):
        patterns = (
            r"(?<![A-Za-z])[A-Za-z]:[\\/]",
            r"(?:/Users/|/home/|\\\\[A-Za-z0-9_-]+\\)",
            r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----",
            r"\b(?:ghp_|github_pat_|sk-proj-)[A-Za-z0-9_]{20,}",
        )
        for path in SKILLS.rglob("*"):
            if not path.is_file():
                continue
            self.assertFalse(path.is_symlink())
            self.assertIn(path.suffix, {".md", ".yaml"})
            text = path.read_text(encoding="utf-8")
            for pattern in patterns:
                self.assertIsNone(re.search(pattern, text), f"Review publication content: {path}")

    def test_private_case_study_is_explicitly_excluded(self):
        vehicle = SKILLS / "prepare-simready-vehicle"
        self.assertFalse((vehicle / "references/highway-sim-case-study.md").exists())
        self.assertIn("private vehicle case study is deliberately not distributed",
                      (vehicle / "SKILL.md").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
