#!/bin/bash

# ROS MCP Client Setup Script
# Sets up the environment for the Gemini Live client

set -e  # Exit on any error

echo "🔧 ROS MCP Client Setup"
echo "======================="

# Check if we're in WSL
if grep -q Microsoft /proc/version 2>/dev/null; then
    echo "🐧 WSL environment detected"
    IS_WSL=true
else
    echo "🐧 Native Linux environment"
    IS_WSL=false
fi

# Install system dependencies
echo "📦 Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y python3-dev python3-venv

# Install audio dependencies only if not in WSL
if [ "$IS_WSL" = false ]; then
    echo "🎵 Installing audio dependencies..."
    sudo apt-get install -y portaudio19-dev
fi

# Install Python dependencies using uv
echo "🐍 Installing Python dependencies..."
if command -v uv &> /dev/null; then
    echo "Using uv for dependency management..."
    uv sync
else
    echo "uv not found, using pip..."
    pip install -e .
fi

# Create .env file if it doesn't exist
if [ ! -f .env ]; then
    echo "📝 Creating .env template..."
    cat > .env << EOF
# Google API Key for Gemini
GOOGLE_API_KEY=your_google_api_key_here
EOF
    echo "⚠️  Please edit .env and add your Google API key"
fi

echo ""
echo "✅ Setup complete!"
echo ""
echo "📝 Next steps:"
echo "1. Add your Google API key to .env file"
echo "2. Verify mcp_config.json points to your ros-mcp-server"
echo "3. Run: python gemini_client.py"
echo ""
if [ "$IS_WSL" = true ]; then
    echo "💡 WSL Notes:"
    echo "- This client runs in text-only mode (no audio/video)"
    echo "- Use WSL paths in mcp_config.json (e.g., /mnt/c/Users/...)"
fi

