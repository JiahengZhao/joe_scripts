#!/usr/bin/env zsh

SRC_DIR="src"
UPDATED=()
SKIPPED=()

# Ensure src folder exists
if [ ! -d "$SRC_DIR" ]; then
  echo "❌ Folder '$SRC_DIR' not found!"
  exit 1
fi

echo "🔍 Searching for git repositories in '$SRC_DIR'..."

# Find all repos (folders containing .git)
find "$SRC_DIR" -type d -name ".git" | while read -r gitdir; do
  repo=$(dirname "$gitdir")
  echo ""
  echo "📁 Checking repo: $repo"
  cd "$repo" || continue

  # Check if working tree is clean
  if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "⚠️  Skipping (unstaged or uncommitted changes)"
    SKIPPED+=("$repo")
    cd - >/dev/null || exit
    continue
  fi

  # Check for merge conflicts
  if git ls-files -u | grep -q .; then
    echo "⚠️  Skipping (merge conflicts present)"
    SKIPPED+=("$repo")
    cd - >/dev/null || exit
    continue
  fi

  # Fetch and pull updates
  echo "⬇️  Pulling latest changes..."
  if git pull --ff-only; then
    UPDATED+=("$repo")
  else
    echo "⚠️  Pull failed (non-fast-forward or error)"
    SKIPPED+=("$repo")
  fi

  cd - >/dev/null || exit
done

echo ""
echo "✅ Update complete."
echo "-----------------------------"
echo "✔️  Updated repos:"
for r in "${UPDATED[@]}"; do echo "  - $r"; done

echo ""
echo "⏸️  Skipped repos:"
for r in "${SKIPPED[@]}"; do echo "  - $r"; done