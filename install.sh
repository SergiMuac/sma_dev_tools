#!/bin/bash

# --- Configuration ---
REPO="SergiMuac/sma_dev_tools"  # <--- CHANGE THIS
BINARY_NAME="sma"
VERSION="0.1.0"
INSTALL_DIR="$HOME/.local/bin"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Installing $BINARY_NAME...${NC}"

# 1. Ensure the install directory exists
mkdir -p "$INSTALL_DIR"

# 2. Determine download URL
# This fetches the "latest" release artifact.
# NOTE: Ensure you upload the file named simply 'sma' (for Linux) to the release.
DOWNLOAD_URL="https://github.com/$REPO/releases/download/$VERSION/$BINARY_NAME"

echo -e "Downloading latest version from $DOWNLOAD_URL..."

# 3. Download the binary
if command -v curl >/dev/null 2>&1; then
    curl -L --fail "$DOWNLOAD_URL" -o "$INSTALL_DIR/$BINARY_NAME"
elif command -v wget >/dev/null 2>&1; then
    wget -O "$INSTALL_DIR/$BINARY_NAME" "$DOWNLOAD_URL"
else
    echo -e "${RED}Error: Neither curl nor wget found.${NC}"
    exit 1
fi

# 4. Make executable
chmod +x "$INSTALL_DIR/$BINARY_NAME"

# 5. Check PATH
if [[ ":$PATH:" != *":$INSTALL_DIR:"* ]]; then
    echo -e "${RED}Warning: $INSTALL_DIR is not in your PATH.${NC}"
    echo "Add the following line to your shell config (.bashrc / .zshrc):"
    echo "  export PATH=\"\$HOME/.local/bin:\$PATH\""
fi

# 6. Install Completion (Optional but recommended)
# We try to install completion automatically
"$INSTALL_DIR/$BINARY_NAME" --install-completion > /dev/null 2>&1

echo -e "${GREEN}Success! $BINARY_NAME has been installed to $INSTALL_DIR${NC}"
echo -e "Run '${GREEN}$BINARY_NAME --help${NC}' to get started."