# Gemini Client for ROS MCP Server

A simplified, text-only version of the Gemini Live client for controlling ROS robots through natural language commands. This version is optimized for WSL and environments where audio/video features are not needed.

## Features

- ✅ **Text-only interaction** - No audio or video dependencies
- ✅ **WSL compatible** - Works perfectly in Windows Subsystem for Linux
- ✅ **Minimal dependencies** - Only essential packages required
- ✅ **ROS robot control** - Full access to all MCP server tools
- ✅ **Cross-platform** - Works on Linux, macOS, and Windows

## Quick Setup

### 1. Automated Setup (Recommended)

```bash
# Navigate to the gemini_live folder
cd clients/gemini_live

# Run the automated setup script
chmod +x setup_gemini_client.sh
./setup_gemini_client.sh
```

The setup script will:
- ✅ Install system dependencies (python3-dev, python3-venv)
- ✅ Install Python dependencies using `uv sync` or `pip install -e .`
- ✅ Create `.env` template in the correct location
- ✅ Provide clear next steps to import the `GOOGLE_API_KEY`

### 2. Manual Setup (Alternative)

If you prefer manual setup:

```bash
# Install dependencies
uv sync  # or pip install -e .

# Create .env file
echo "GOOGLE_API_KEY=your_actual_api_key_here" > .env
```

### 3. Get Google API Key

1. Go to [Google AI Studio](https://aistudio.google.com/)
2. Sign in with your Google account
3. Click "Get API key" and create a new key
4. Copy the API key

### 4. Configure Environment

**Location:** The `.env` file must be in `clients/gemini_live/.env`

```bash
# Edit the .env file
nano clients/gemini_live/.env
```

Add your Google API key:
```bash
# Google API Key for Gemini
GOOGLE_API_KEY=your_actual_api_key_here
```

### 5. Configure MCP Server

Update `mcp.json` with the correct path to your ros-mcp-server:
```json
{
   "mcpServers": {
     "ros-mcp-server": {
       "name": "ROS-MCP Server (Linux/WSL)",
       "transport": "stdio",
       "command": "uv",
       "args": [
         "--directory",
         "/mnt/c/Users/<USERNAME>/ros-mcp-server",
         "run",
         "server.py",
         "--transport=stdio"
       ]
     }
   }
}
```

**For WSL users:** Use WSL paths (e.g., `/mnt/c/Users/<USERNAME>/ros-mcp-server`)

## Usage

### Start the Client

```bash
# Navigate to the client directory
cd clients/gemini_live

# Run the client
uv run gemini_client.py

# Or with Python directly
python gemini_client.py
```

### Example Session

```
🚀 Starting Gemini Live Text-Only Client...
💡 Type 'q' to quit
==================================================
✅ Connected to MCP server
🔧 Loaded 25 tools from MCP server
✅ Connected to Gemini Live
🎯 Ready for robot control commands!
==================================================
message > connect to robot
response > Successfully connected
message > what topics are available?
response > Here are the available ROS topics:
- /cmd_vel
- /odom
- /scan
message > move the robot forward at 1 m/s
response > I'll move the robot forward at 1 m/s
message > q
Goodbye!
```


## Test with Turtlesim

### Start ROS Environment

```bash
# Terminal 1: Launch rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
# Terminal 2: Start turtlesim
ros2 run turtlesim turtlesim_node
```

### Try These Commands

- "Connect to the robot on localhost"
- "What ROS topics are available?"
- "Move the turtle forward at 1 m/s and 0 rad/s"
- "Rotate the turtle at 3 rad/s"
- "Change the pen color to red"

## Troubleshooting

### Common Issues

**Connection errors?**
- Verify rosbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check MCP server path in `mcp.json`
- Ensure ros-mcp-server is installed and working

**API key errors?**
- Verify `.env` file exists in `clients/gemini_live/.env` with correct key
- Check key is active in Google AI Studio
- Run: `cat clients/gemini_live/.env` to verify the file exists

**Import errors?**
- Run the setup script: `./setup_gemini_client.sh`
- Install manually: `uv sync` or `pip install -e .`

**MCP configuration errors?**
- Check `mcp.json` exists in `clients/gemini_live/`
- Verify the path to ros-mcp-server is correct
- Ensure the ros-mcp-server is running

### WSL-Specific Tips

- Use WSL paths in `mcp.json` (e.g., `/mnt/c/Users/username/...`)
- This version has no audio/video dependencies, so it works perfectly in WSL
- No need for portaudio, pyaudio, or camera libraries

## Comparison with Full Version

| Feature | Lite Version | Full Version |
|---------|-----------|--------------|
| Text interaction | ✅ | ✅ |
| Audio input | ❌ | ✅ |
| Audio output | ❌ | ✅ |
| Camera | ❌ | ✅ |
| Screen capture | ❌ | ✅ |
| WSL compatibility | ✅ | ⚠️ |
| Dependencies | Minimal | Full |
| Resource usage | Low | High |

## Benefits of Text-Only Version

1. **WSL Perfect**: No audio/video dependencies that cause issues in WSL
2. **Lightweight**: Minimal resource usage and dependencies
3. **Reliable**: Fewer potential points of failure
4. **Fast**: Quick startup and response times
5. **Universal**: Works on any system with Python

## Contributing

This text-only version is perfect for:
- WSL users who want reliable robot control
- Server environments without audio/video
- CI/CD pipelines for robot testing
- Educational environments
- Users who prefer text-based interaction

---

**Note**: This is a simplified version of the full Gemini Live client. For audio/video features, use the main `gemini_client.py` with proper audio setup.

