# Gemini Live Text-Only Client for ROS MCP Server

A simplified, text-only version of the Gemini Live client for controlling ROS robots through natural language commands. This version is optimized for WSL and environments where audio/video features are not needed.

## Features

- ✅ **Text-only interaction** - No audio or video dependencies
- ✅ **WSL compatible** - Works perfectly in Windows Subsystem for Linux
- ✅ **Minimal dependencies** - Only essential packages required
- ✅ **ROS robot control** - Full access to all MCP server tools
- ✅ **Cross-platform** - Works on Linux, macOS, and Windows

## Quick Setup

### 1. Install Dependencies

```bash
# Navigate to the gemini_live folder
cd examples/2_gemini/gemini_live

# Run the automated setup
chmod +x setup_text_only.sh
./setup_text_only.sh
```

### 2. Configure Environment

Create a `.env` file with your Google API key:
```bash
echo "GOOGLE_API_KEY=your_google_api_key_here" > .env
```

### 3. Configure MCP Server

Update `mcp_config.json` with the correct path to your ros-mcp-server:
```json
{
   "mcpServers": {
      "ros-mcp-server": {
         "command": "uv",
         "args": [
         "--directory",
         "/path/to/ros-mcp-server", 
         "run",
         "server.py"
         ]
      }
   }
}
```

**For WSL users:** Use WSL paths (e.g., `/mnt/c/Users/username/ros-mcp-server`)

## Usage

### Start the Text-Only Client

```bash
# Basic usage
uv run gemini_client_text_only.py

# With specific response mode
uv run gemini_client_text_only.py --responses=TEXT
```

### Command Line Options

- `--responses`: Response format from Gemini
  - `TEXT` (default): Text responses only
  - `AUDIO`: Audio responses (requires audio setup)

### Example Session

```
🚀 Starting Gemini Live Text-Only Client...
✅ Connected to MCP server
🔧 Loaded 25 tools from MCP server
✅ Connected to Gemini Live
🎯 Ready for robot control commands!
==================================================
🤖 message > connect to robot on localhost
🤖 > Successfully connected
🤖 message > what topics are available?
🤖 > Here are the available ROS topics:
- /cmd_vel
- /odom
- /scan
🤖 message > move the robot forward at 1 m/s
🤖 > I'll move the robot forward at 1 m/s
🤖 message > q
👋 Goodbye!
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
- Check MCP server path in `mcp_config.json`
- Ensure ros-mcp-server is installed and working

**API key errors?**
- Verify `.env` file exists with correct key
- Check key is active in Google AI Studio

**Import errors?**
- Run the setup script: `./setup_text_only.sh`
- Install manually: `uv pip install google-genai python-dotenv mcp`

### WSL-Specific Tips

- Use WSL paths in `mcp_config.json` (e.g., `/mnt/c/Users/username/...`)
- This version has no audio/video dependencies, so it works perfectly in WSL
- No need for portaudio, pyaudio, or camera libraries

## Comparison with Full Version

| Feature | Text-Only | Full Version |
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

