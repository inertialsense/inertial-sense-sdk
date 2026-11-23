---
name: pre-pr-check
description: Check a branch against the SDK code style guide and the repo's PR template before opening a PR or pushing to one. Covers doc-block presence and form for every added or changed function and member, file headers and copyright years, clang-format on changed lines, naming, and PR-template items. Reports counts per item, and anything unchecked blocks the push. Run it before every `gh pr create` and before every push to a branch with an open PR, in every C/C++ repo (SDK, imx, is-imx, is-gpx, is-common, Logalyzer). Argument is optional: the base ref (default origin/develop).
---

# Pre-PR check — `/pre-pr-check [<base-ref>]`

Run this before `gh pr create`, and again before every push to a branch with an open PR. It is the style gate. The Set B tests are a separate gate, and you need both.

**Why this exists (Kyle, 2026-11-22).** Agents kept reporting style compliance after grepping their added lines for forbidden forms (`///`, `/*! */`). That grep checks syntax, not whether a doc block exists. On imx#1835 it missed 29 function declarations documented with a trailing `//!<` and more than a dozen undocumented functions, and an agent wrongly said clang-format wasn't installed, leaving 27 lines over 200 columns. A check that finds no forbidden forms does not show that the code complies.

**Rules for this check:**
- **Count, don't assert.** Every item below ends in a number, such as "41 functions added or changed, 41 documented, 0 malformed". "Looks fine" doesn't count.
- **Unchecked means blocked.** If you can't do an item, stop and say so. Don't push, and don't move it to a "not done" list in the PR description. Fix the obstacle or ask.
- **The style guide is normative.** Read `code-style-guide.md` (at the root of this repo) at the start of this check, every time. This skill summarizes it, and where they differ the guide wins. For formatting, the repo's `.clang-format` wins over both.
- **Scope is the branch's diff.** Check what this branch adds or changes. Don't reformat or re-document code you didn't touch, except as the copyright and file-header rules below require for files you touched.

## Step 0 — Set up the diff

Run each step from the root of each repo or submodule that has changes, on its own branch. An SDK change inside imx is checked in the SDK checkout against the SDK's own base and `.clang-format`.

```bash
BASE=${1:-origin/develop}
git fetch origin
MB=$(git merge-base "$BASE" HEAD)
git diff --name-only --diff-filter=A "$MB" HEAD -- '*.h' '*.hpp' '*.c' '*.cpp'   # new files
git diff --name-only --diff-filter=M "$MB" HEAD -- '*.h' '*.hpp' '*.c' '*.cpp'   # modified files
```

Write both lists down. Every later step reports against them.

## Step 1 — Doc blocks: presence and form (style guide §2)

The forbidden-form grep is the easy part. The main job is to confirm that each documentable item **has** a doc block and that the block is correct.

1. **Inventory.** List every function, method, class, struct, enum, member variable and enumerator that the diff **adds** or whose **signature changes**. Get candidates mechanically, then read every hunk to complete the list, because the regex will miss some and over-match others:
   ```bash
   git diff -U0 "$MB" HEAD -- '*.h' '*.hpp' '*.c' '*.cpp' | grep -E '^\+[^+].*\b[A-Za-z_][A-Za-z0-9_:<>,*& ]*\s*\(' | grep -vE '^\+\s*(//|\*|return|if|for|while|switch)\b'
   ```
   Read each changed header hunk in full, and each `.cpp` hunk that defines a function not declared in a header.
2. **Check each item** against the guide:
   - Functions, methods, classes, structs and enums get a **preceding** `/** */` block. A function's block has a brief, an `@param` for each parameter, and an `@return` unless it returns `void`. A trailing `//!<` on a function is **malformed**: §2.4 reserves it for members, enumerators and parameters.
   - Member variables and enumerators get a trailing `//!<` or a preceding `/** */`.
   - Public API, meaning anything in a header that callers use, **MUST** be documented (§2.7). Non-obvious private functions, invariants, ownership, locking and units **SHOULD** be. Trivial getters and setters are optional.
   - A changed signature **MUST** update its doc block in the same change, covering params added, removed or renamed and the return. Check that the names in `@param` match the actual parameters.
   - After moves and renames, check that doc text doesn't still name old identifiers.
3. **Forbidden forms** (§2.2), in added lines only:
   ```bash
   git diff -U0 "$MB" HEAD -- '*.h' '*.hpp' '*.c' '*.cpp' | grep -nE '^\+.*(///|/\*!)' ; \
   git diff -U0 "$MB" HEAD -- '*.h' '*.hpp' '*.c' '*.cpp' | grep -nE '^\+\s*//![^<]'
   ```
4. Fix every gap, then repeat steps 1–3 on the fixed tree.

**Report:** `Doc blocks: N items added or changed (H in headers, C in .cpp); N documented; 0 missing; 0 malformed; 0 forbidden forms.`

## Step 2 — File headers and copyright (style guide §2.5)

- **New files:** the file opens with a `/** */` block containing `@file <name>`, a real one-line `@brief`, `@author <name> on <date>`, and `@copyright Copyright (c) <year> Inertial Sense, Inc. All rights reserved.`. No placeholders.
- **Modified files:** bring the copyright year up to the current year by extending it to a range (`2023` becomes `2023-2026`, and `2021-2024` becomes `2021-2026`). Leave `@author` alone.
- **Modified files with no header block:** add one, with `@file`, a `@brief` describing what the file does, and the copyright line.

```bash
for f in $(git diff --name-only --diff-filter=AM "$MB" HEAD -- '*.h' '*.hpp' '*.c' '*.cpp'); do echo "== $f"; head -12 "$f" | grep -nE '@file|@brief|@author|[Cc]opyright' || echo "   NO HEADER"; done
```

**Report:** `Headers: A new files, all with complete headers; M modified files, M with the current copyright year, K headers added.`

## Step 3 — clang-format (style guide §1)

clang-format is installed in the Hephaestus bridge image at **`/opt/zephyr-venv/bin/clang-format`**, alongside `clang-format-diff.py` and `git-clang-format`. That directory isn't on `PATH`, so call the tools by full path. Don't report clang-format as unavailable without checking there. On a developer machine use that machine's own clang-format (`command -v clang-format`), and if there is none, run the check in a bridge container.

Use this repo's `.clang-format`. The SDK and imx have different ones; if you check an SDK change from inside imx, run the check in the SDK checkout. If a repo has no `.clang-format`, say so in the report and don't apply another repo's.

```bash
CF=/opt/zephyr-venv/bin
# New files: format the whole file with the repo's own style (include regrouping is fine).
$CF/clang-format -i --style=file <new files...>

# Modified files: changed lines only, with include sorting and regrouping turned off so the include block is not reshuffled.
{ sed -e 's/IncludeBlocks: *Regroup/IncludeBlocks: Preserve/' -e '/^SortIncludes/d' .clang-format; echo; echo 'SortIncludes: Never'; } > /tmp/cf-preserve.yml
git diff -U0 "$MB" -- <modified files...> | $CF/clang-format-diff.py -p1 -i -binary $CF/clang-format -style=file:/tmp/cf-preserve.yml

# Re-check: prints nothing and exits 0 once clean.
git diff -U0 "$MB" -- <modified files...> | $CF/clang-format-diff.py -p1 -binary $CF/clang-format -style=file:/tmp/cf-preserve.yml
```

Then confirm no added line is over the column limit (200 for the SDK and imx):
```bash
git diff -U0 "$MB" -- '*.h' '*.hpp' '*.c' '*.cpp' | grep -E '^\+[^+]' | awk 'length($0) > 201' | wc -l
```

Review the formatting changes before committing them. They're part of your diff. If clang-format-diff.py reformats lines you didn't write, which happens when an inserted declaration sits next to aligned members, don't accept it: move your insertion (for example to the end of the class, after a blank line) or apply only the hunks for your own lines.

A renamed or moved file must be passed with both its old and its new path (`git diff -U0 -M "$MB" -- <old> <new>`). With only the new path git shows the whole file as added, and `clang-format-diff.py -i` then reformats all of it, burying the real change. Check `git diff --stat` after formatting: a file with far more changed lines than you wrote was reformatted whole, so revert it and run again with both paths.

**Report:** `clang-format: N new files formatted whole, M modified files formatted on changed lines; re-check clean; 0 added lines over 200 columns.`

## Step 4 — Naming and language rules (style guide §3–4)

Read the added code against these rules and fix anything that breaks them:
- `UPPER_SNAKE_CASE` constants and macros. `PascalCase` types and `enum class` enumerators. `camelCase` functions and locals. Trailing `_` on private members in new code, but follow the existing file's style rather than mass-renaming. No `m_` or `k` prefixes and no `I` interface prefixes in new code.
- TU-local helpers go in an unnamed namespace, not `static`. RAII and smart pointers, no raw owning `new`. `std::lock_guard`, never bare `lock()`/`unlock()`. `nullptr`, `override`, and `= delete` for non-copyable types.
- Heavy includes go in the `.cpp`, and the public header uses forward declarations.
- No lock held across a callback or blocking I/O, and no network or port I/O on the GUI thread.
- Named `constexpr` for timeouts, intervals, retry counts and buffer sizes, not magic numbers.
- Don't create a new source file for a single function.

**Report:** the findings and fixes, or `Naming/language: read all N hunks, 0 issues.`

## Step 5 — PR template and repo conventions

- This repo has no `.github/pull_request_template.md`, so there is no template to meet. The PR description carries the count report from step 6.
- Run `git diff "$MB" HEAD | grep -n '/tmp/'` and treat any hit in a build file as a hard stop (see "Build hygiene" in `~/.claude/CLAUDE.md`).

**Report:** `PR template: <item> done, <item> done.`

## Step 6 — Re-run tests and report

The fixes change code, so rebuild and re-run the tests for the modules you touched (Set A and Set B) before committing. Commit the fixes as their own commit, for example `Bring <area> into line with the code style guide`.

Put the report in your reply. Also put it in the PR description, or in a PR comment if you're pushing to an open PR:

```
Pre-PR check (base <MB short sha>):
- Doc blocks: 41 added or changed (29 headers, 12 .cpp); 41 documented; 0 missing; 0 malformed; 0 forbidden forms
- Headers: 3 new files complete; 7 modified files with copyright 2026; 1 header added
- clang-format: 3 new files whole, 7 modified on changed lines; re-check clean; 0 lines over 200
- Naming/language: read all 38 hunks; 2 fixed (static helper moved to unnamed namespace; magic 2500 -> PORT_SCAN_TIMEOUT_MS)
- PR template: none in this repo
- Tests: test_gnss_unit 78 passed; test_gnss_receiver_profile 7 passed
```

If any line reads anything other than clean or done, don't open or push the PR.
