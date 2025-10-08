#!/usr/bin/env bash
# ROS MCP Client Setup Script
# Sets up the environment for the Gemini Live client

set -e  # Exit on any error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENT_DIR="$SCRIPT_DIR"

echo "🔧 ROS MCP Client Setup"
echo "======================="
echo "📁 Client directory: $CLIENT_DIR"

# Detect operating system
OS="$(uname -s)"
case "${OS}" in
    Linux*)     MACHINE=Linux;;
    Darwin*)    MACHINE=Mac;;
    CYGWIN*)    MACHINE=Cygwin;;
    MINGW*)     MACHINE=MinGw;;
    *)          MACHINE="UNKNOWN:${OS}"
esac

echo "🖥️  Detected OS: $MACHINE"

# Install system dependencies based on OS
echo "📦 Installing system dependencies..."
case $MACHINE in
    Linux)
        if command -v apt-get &> /dev/null; then
            sudo apt-get update -qq
            sudo apt-get install -y -qq python3-dev python3-venv
        elif command -v yum &> /dev/null; then
            sudo yum install -y python3-devel python3-venv
        elif command -v pacman &> /dev/null; then
            sudo pacman -S --noconfirm python python-virtualenv
        else
            echo "⚠️  Unknown Linux package manager. Please install python3-dev and python3-venv manually."
        fi
        ;;
    Mac)
        # Check if Homebrew is installed
        if command -v brew &> /dev/null; then
            echo "🍺 Using Homebrew to install dependencies..."
            brew install python@3.10 || brew install python@3.11 || brew install python@3.12
        else
            echo "⚠️  Homebrew not found. Please install Python 3.10+ manually or install Homebrew first."
            echo "   Visit: https://brew.sh/"
        fi
        ;;
    *)
        echo "⚠️  Unsupported OS: $MACHINE"
        echo "   Please ensure Python 3.10+ is installed manually."
        ;;
esac

# Change to project root for dependency installation
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
echo "📁 Project root: $PROJECT_ROOT"
cd "$PROJECT_ROOT"

# Install Python dependencies using uv
echo "🐍 Installing Python dependencies..."
if command -v uv &> /dev/null; then
    echo "Using uv for dependency management..."
    uv sync
else
    echo "uv not found, using pip..."
    pip install -e .
fi

# Create .env file in the client directory
ENV_FILE="$CLIENT_DIR/.env"
if [ ! -f "$ENV_FILE" ]; then
    echo "📝 Creating .env template in $CLIENT_DIR..."
    cat > "$ENV_FILE" << EOF
# Google API Key for Gemini
# Get your API key from: https://aistudio.google.com/
GOOGLE_API_KEY=your_google_api_key_here
EOF
    echo "⚠️  Please edit $ENV_FILE and add your Google API key"
else
    echo "✅ .env file already exists in $CLIENT_DIR"
fi

echo ""
echo "✅ Setup complete!"
echo ""
echo "📝 Next steps:"
echo "1. Add your Google API key to $ENV_FILE"
echo "2. Verify mcp_config.json points to your ros-mcp-server"
case $MACHINE in
    Mac|Linux)
        echo "3. Run: cd $CLIENT_DIR && uv run gemini_client.py"
        ;;
    *)
        echo "3. Run: cd $CLIENT_DIR && python gemini_client.py"
        ;;
esac
echo ""

# Platform-specific notes
case $MACHINE in
    Mac)
        echo "🍎 macOS Notes:"
        echo "- If you encounter permission issues, you may need to run with 'python3' instead of 'python'"
        echo "- Ensure Xcode Command Line Tools are installed: xcode-select --install"
        ;;
    Linux)
        # Check if running in WSL
        if grep -qi microsoft /proc/version 2>/dev/null; then
            echo "� WSL Notes:"
            echo "- This client runs in text-only mode (no audio/video)"
            echo "- Use WSL paths in mcp_config.json (e.g., /mnt/c/Users/...)"
        else
            echo "🐧 Linux Notes:"
            echo "- Make sure your user has proper permissions for the project directory"
        fi
        ;;
    *)
        echo "⚠️  Platform-specific notes not available for $MACHINE"
        ;;
esac
