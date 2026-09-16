#!/usr/bin/env bash
# Installs a JDK (if missing) and builds ENHSP from source into tools/enhsp.
# Re-run safely at any time - it skips steps that are already done.
#
# Linux/WSL2 equivalent of scripts/setup_enhsp.ps1 - keep both in sync if
# either changes.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENHSP_DIR="$REPO_ROOT/tools/enhsp"

# 1. Java - matches the .ps1's choice of Temurin 17
if ! command -v java >/dev/null 2>&1; then
    echo "No Java found - installing OpenJDK 17..."
    sudo apt update
    sudo apt install -y openjdk-17-jdk
fi
echo "Using Java: $(command -v java)"
java -version

# 2. Clone ENHSP if not already present
if [ ! -d "$ENHSP_DIR" ]; then
    echo "Cloning ENHSP into $ENHSP_DIR ..."
    git clone --depth 1 https://github.com/hstairs/enhsp "$ENHSP_DIR"
fi

# 3. Build it, if the jar doesn't already exist
JAR="$ENHSP_DIR/enhsp-dist/enhsp.jar"
if [ -f "$JAR" ]; then
    echo "ENHSP already built at $JAR"
else
    echo "Building ENHSP (bash compile)..."
    (cd "$ENHSP_DIR" && bash compile)
    if [ ! -f "$JAR" ]; then
        echo "Build finished but $JAR was not produced - see the compile output above." >&2
        exit 1
    fi
    echo "Built $JAR"
fi
