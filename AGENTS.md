# Repository Guidelines

## Project structure & module organization

This repository is a computer science knowledge base containing Markdown notes, diagrams, and independent examples.
Topic directories include `Cpp/`, `Go/`, `Linux/`, `Embedded/`, and `Robotics/`.
Code lives in topic-specific directories such as `Cpp/code/` and `Go/proj-go/`; images commonly live in `figures/`.
Tests belong to individual examples, such as `Linux/CMake/proj-cpp/tests/`.
`Blogs/` contains the Hexo site. Root scripts and `.blog-sync.json` control synchronization from notes to blog posts.

## Build, test, and development commands

There is no repository-wide build. Run commands within the relevant project and check its dependencies first.

- In `Blogs/`, run `npm install` to install dependencies; CI uses Node.js 20 and Python 3.11.
- `npm run sync` regenerates synchronized posts using `add_frontmatter.py --clean`.
- `npm run build:ci` cleans and generates the static site; `npm run server` starts a local preview.
- From the repository root, `cmake -S Linux/CMake/proj-cpp -B Linux/CMake/proj-cpp/build`
  configures the C++ example; `cmake --build Linux/CMake/proj-cpp/build` builds it.
  Its tests require GoogleTest shared libraries in the project's `libs/` directory.
- Run `go test ./...` inside the affected directory containing `go.mod`.

## Coding style & naming conventions

Follow `.editorconfig`: UTF-8, LF endings, final newlines, and two-space indentation; Makefiles use tabs.
Use `gofmt` for Go code. Format C++ with `clang-format -i --style=file <file>` and inspect with `.clang-tidy`.
C++ naming uses `CamelCase` classes/functions, `lower_case` variables, trailing underscores for private members,
and `kCamelCase` constants. Use braces around control-flow bodies.
Follow `convention.md` for notes: sentence-case English headings, lines at most 118 characters,
and spaces between Chinese text and English words. Preserve nearby filename patterns.

## Testing guidelines

There is no global coverage threshold or unified test runner. Go examples use `testing`, with Testify and gomock
in some projects; name tests `*_test.go` with `Test...` functions. C++ examples include GoogleTest;
run `Linux/CMake/proj-cpp/build/bin/sample_test` after building.
For documentation changes, check links, image paths, and rendered output. The Node server's `npm test` is a placeholder.

## Commit & pull request guidelines

Recent commits use `feat:`, `fix:`, and `docs:`, optionally scoped, for example `docs(Robotics): update rviz instructions`.
Keep changes focused. PRs should describe affected topics, explain the change, list validation and prerequisites,
link relevant issues, and include screenshots for visual changes.

## Blog synchronization

Edit original notes and assets rather than generated copies under `Blogs/source/_posts/synced`.
Update `.blog-sync.json` when changing synchronization scope. Pushes to `main` or `master` trigger blog build and deployment.
