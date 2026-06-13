## What this repo is

A personal CS knowledge base (notes, code examples, and reference material) spanning hardware, systems, languages, \
databases, cloud-native, and AI topics. The primary output is a Hexo static blog, \
auto-synced from the Markdown files in this repo.

## Blog / sync workflow

### Sync Markdown notes → Hexo posts

```bash
# Sync (with stale-file cleanup)
python add_frontmatter.py --clean

# Dry-run: see what would change without writing
python add_frontmatter.py --dry-run

# Sync without removing stale posts
python add_frontmatter.py
```

The script reads `.blog-sync.json` to decide which directories to sync. Output lands in `Blogs/source/_posts/synced/`.

### Local blog dev server

```bash
cd Blogs
npm install          # first time only
npm run server       # starts Hexo at http://localhost:4000
```

### Build the static site

```bash
cd Blogs
npm run build:ci     # hexo clean + hexo generate → ./public/
```

### CI / deploy

Pushing to `master` triggers GitHub Actions (`.github/workflows/`), which runs `add_frontmatter.py --clean`, \
builds the Hexo site, and deploys to `JohnJeep/JohnJeep.github.io` via `PAGES_REPO_TOKEN`.

## Repository structure

| Directory | Content |
|---|---|
| `Blogs/` | Hexo site (theme: Butterfly). `source/_posts/synced/` is auto-generated — do not edit by hand. |
| `add_frontmatter.py` | Blog sync script; reads `.blog-sync.json` for include/exclude rules. |
| `.blog-sync.json` | Whitelist of directories to sync and `category_aliases` for display names. |
| `convention.md` | Chinese/English typesetting conventions used in all notes. |
| `Go/proj-go/` | Runnable Go examples organised by topic (goroutine, gin, sqlc, etc.). |
| `Cpp/code/` | C++ example code organised by topic. |
| `C/code/` | C example code. |

All other top-level directories (`AI`, `Architecture`, `Assembly`, `CloudNative`, …) are pure Markdown notes.

## Sync config rules (`.blog-sync.json`)

- `include_roots` — directories whose `.md` files are synced.
- `exclude_prefixes` — path prefixes skipped during sync (e.g. `Blogs/`, `Go/proj-go/`).
- `exclude_file_names` — filenames never synced (e.g. `README.md`).
- `category_aliases` — maps directory names to display category names (e.g. `"Cpp" → "C++"`).
- `output_subdir` — destination inside the Hexo source tree.

## Document conventions

From `convention.md`:

---

## Coding behavior guidelines

Derived from Andrej Karpathy's observations on LLM coding pitfalls. These apply whenever writing or editing code \
in this repo (`add_frontmatter.py`, Go/Cpp/C examples, Hexo config, shell scripts, etc.).

**Tradeoff:** These guidelines bias toward caution over speed. Use judgment for trivial one-liners.

### 1. Think before coding

Don't assume. Don't hide confusion. Surface tradeoffs.

Before implementing:
- State assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them — don't pick silently.
- If a simpler approach exists, say so and push back when warranted.
- If something is unclear, stop, name what's confusing, and ask.

### 2. Simplicity first

Minimum code that solves the problem. Nothing speculative.

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If 200 lines could be 50, rewrite it.

Ask: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

### 3. Surgical changes

Touch only what you must. Clean up only your own mess.

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it — don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that **your** changes made unused.
- Don't remove pre-existing dead code unless asked.

Every changed line should trace directly to the user's request.

### 4. Goal-driven execution

Define success criteria. Loop until verified.

Transform tasks into verifiable goals:
- "Fix the sync bug" → "Write a test that reproduces it, then make it pass"
- "Refactor `add_frontmatter.py`" → "Ensure behavior is identical before and after"

For multi-step tasks, state a brief plan first:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Weak criteria ("make it work") require constant clarification — avoid them.
