# CLAUDE.md

Guidance for Claude Code in this repository.

## Code style

`code-style-guide.md` is normative: read it before writing or changing C/C++ here. The repo's `.clang-format` is the formatting source of truth.

## Before opening a PR or pushing to one

Run `/pre-pr-check` (`.claude/skills/pre-pr-check/SKILL.md`) and fix everything it finds. It counts doc blocks (every added or changed function needs a preceding `/** */` block with `@param` and `@return`; a trailing `//!<` is for members only), file headers, clang-format on changed lines and naming, and its report goes in the PR description.

## File headers and copyright

- A new file opens with the full file block of the style guide (§2.5): `@file`, a real `@brief`, `@author <name> on <date>`, and `@copyright Copyright (c) <year> Inertial Sense, Inc. All rights reserved.`
- A file you modify has its copyright year extended to a range ending this year, for example `2023` becomes `2023-2026`.
- A file you touch that has no header gets one.

## clang-format

In the Hephaestus bridge image it is at `/opt/zephyr-venv/bin/` (`clang-format`, `clang-format-diff.py`, `git-clang-format`), which is not on `PATH`. A developer machine uses its own clang-format. Format new files whole and modified files on changed lines only; the check's step 3 has the commands.

`clang-format-diff.py -i` can reformat existing lines next to yours, for example the aligned members beside a declaration you inserted. Read the hunks it reports: if it touches lines you did not write, move your insertion (for example to the end of the class after a blank line) or apply only the hunks for your own lines. Some `.clang-format` files end without a newline, so the step-3 command that appends `SortIncludes: Never` must put a newline first, as the skill's version does.
