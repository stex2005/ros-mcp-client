#!/usr/bin/env python3
"""
Gemini Live Text-Only Client for ROS MCP Server
A simplified version that only uses text input/output without audio or video.
"""

import argparse
import asyncio
import json
import os
import sys
import traceback

from dotenv import load_dotenv
from google import genai
from google.genai import types
from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

load_dotenv()

# Gemini Live model and default settings
MODEL = "models/gemini-2.5-flash-live-preview"
DEFAULT_RESPONSE_MODALITY = "TEXT"  # Text-only mode

# System instructions to guide Gemini's behavior and tool usage.
system_instructions = """
You have access to the tools provided by ros_mcp_server.
When successfully connected, reply just "Successfully connected".
You can help users control ROS robots through natural language commands.
"""


class GeminiROSClient:
    """
    Unified Gemini Live client for ROS robot control.
    Handles configuration, MCP connection, and Gemini communication in one class.
    """

    def __init__(self, response_modality=DEFAULT_RESPONSE_MODALITY):
        """Initialize the client with configuration and API setup."""
        self.response_modality = response_modality
        self.session = None
        self.mcp_session = None
        
        # Load configuration
        self._load_config()
        self._setup_api_client()

    def _load_config(self):
        """Load and validate MCP configuration."""
        config_path = "mcp.json"
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"mcp.json not found at {config_path}")

        with open(config_path, "r") as f:
            config = json.load(f)

        if "mcpServers" not in config or "ros-mcp-server" not in config["mcpServers"]:
            raise ValueError("Invalid mcp.json: missing 'mcpServers.ros-mcp-server'")

        self.server_config = config["mcpServers"]["ros-mcp-server"]
        
        # Create server parameters for MCP connection
        self.server_params = StdioServerParameters(
            command=self.server_config["command"],
            args=self.server_config["args"]
        )

    def _setup_api_client(self):
        """Setup Gemini API client with authentication."""
        api_key = os.environ.get("GOOGLE_API_KEY")
        if not api_key:
            print("Error: GOOGLE_API_KEY not found!")
            print("Please create a .env file with your Google API key")
            sys.exit(1)
        
        print("API key loaded successfully")
        self.client = genai.Client(
            http_options={"api_version": "v1beta"},
            api_key=api_key,
        )

    def _convert_mcp_tool(self, tool):
        """Convert a single MCP tool to Gemini format."""
        tool_description = {"name": tool.name, "description": tool.description}
        
        if tool.inputSchema.get("properties"):
            tool_description["parameters"] = {
                "type": tool.inputSchema["type"],
                "properties": {},
            }
            
            for param_name, param_schema in tool.inputSchema["properties"].items():
                param_type = self._get_param_type(param_schema)
                param_def = {"type": param_type, "description": ""}
                
                if param_type == "array" and "items" in param_schema:
                    param_def["items"] = {"type": param_schema["items"].get("type", "object")}
                
                tool_description["parameters"]["properties"][param_name] = param_def
            
            if "required" in tool.inputSchema:
                tool_description["parameters"]["required"] = tool.inputSchema["required"]
        
        return tool_description

    def _get_param_type(self, param_schema):
        """Extract parameter type from schema, handling anyOf unions."""
        if "type" in param_schema:
            return param_schema["type"]
        elif "anyOf" in param_schema:
            for type_option in param_schema["anyOf"]:
                if type_option.get("type") != "null":
                    return type_option["type"]
        return "string"  # fallback

    async def send_text(self):
        """
        Handle text input from user and send to Gemini Live session.

        Continuously prompts for user input and sends it to the session.
        Breaks the loop when user types 'q' to quit.
        """
        while True:
            text = await asyncio.to_thread(
                input,
                "message > ",
            )
            if text.lower() == "q":
                break

            await self.session.send_client_content(
                turns={"role": "user", "parts": [{"text": text or "."}]},
                turn_complete=True,
            )

    async def handle_tool_call(self, tool_call):
        """
        Process tool calls from Gemini and execute them via MCP session.

        Args:
            tool_call: Tool call request from Gemini containing function calls
        """
        for function_call in tool_call.function_calls:
            # Execute the tool call through MCP server
            result = await self.mcp_session.call_tool(
                name=function_call.name,
                arguments=function_call.args,
            )

            # Convert MCP result to JSON-serializable format
            result_content = []
            if hasattr(result, "content"):
                for content_item in result.content:
                    # Handle TextContent objects with 'text' attribute
                    if hasattr(content_item, "text"):
                        result_content.append(content_item.text)
                    # Handle objects that can be dumped to dict
                    elif hasattr(content_item, "model_dump"):
                        dumped = content_item.model_dump()
                        if isinstance(dumped, dict):
                            result_content.append(str(dumped))
                        else:
                            result_content.append(str(dumped))
                    # Handle plain strings
                    elif isinstance(content_item, str):
                        result_content.append(content_item)
                    # Handle dicts
                    elif isinstance(content_item, dict):
                        result_content.append(str(content_item))
                    # Fallback to string conversion
                    else:
                        result_content.append(str(content_item))

            # Join content items into a single result string
            result_text = "\n".join(result_content) if result_content else str(result)

            # Format response for Gemini
            function_responses = [
                types.FunctionResponse(
                    name=function_call.name,
                    id=function_call.id,
                    response={"result": result_text},
                )
            ]

            await self.session.send_tool_response(function_responses=function_responses)

    async def receive_responses(self):
        """
        Background task to receive responses from Gemini session.

        Processes text responses and tool calls from Gemini.
        """
        while True:
            turn = self.session.receive()
            turn_text = ""
            first_text = True

            async for response in turn:
                # Handle server content with model turn
                server_content = response.server_content
                if server_content and server_content.model_turn:
                    for part in server_content.model_turn.parts:
                        # Handle text responses
                        if part.text:
                            text_content = part.text
                            if first_text:
                                print(
                                    f"\nresponse > {text_content}", end="", flush=True
                                )
                                first_text = False
                            else:
                                print(text_content, end="", flush=True)
                            turn_text += text_content
                    continue

                # Fallback: Handle text responses from Gemini (for backward compatibility)
                if text_content := response.text:
                    if first_text:
                        print(f"\nresponse > {text_content}", end="", flush=True)
                        first_text = False
                    else:
                        print(text_content, end="", flush=True)
                    turn_text += text_content

                # Handle tool calls from Gemini
                tool_call = response.tool_call
                if tool_call is not None:
                    await self.handle_tool_call(tool_call)

            # Complete the response display
            if turn_text:
                print()  # Add newline after response
                print("message > ", end="", flush=True)  # Show next prompt

    async def run(self):
        """
        Main execution method that sets up and runs the Gemini Live session.

        Connects to MCP server, configures tools, and starts all async tasks
        for text-based communication.
        """
        print("🚀 Starting Gemini Live Text-Only Client...")
        print("💡 Type 'q' to quit")
        print("=" * 50)

        # Connect to MCP server using stdio
        async with stdio_client(self.server_params) as (read, write):
            async with ClientSession(read, write) as mcp_session:
                # Initialize the connection between client and server
                await mcp_session.initialize()
                print("✅ Connected to MCP server")

                # Store MCP session for tool calling
                self.mcp_session = mcp_session

                # Get available tools from MCP server
                available_tools = await mcp_session.list_tools()
                print(f"🔧 Loaded {len(available_tools.tools)} tools from MCP server")

                # Convert MCP tools to Gemini-compatible format
                functional_tools = [self._convert_mcp_tool(tool) for tool in available_tools.tools]

                # Configure Gemini Live tools (MCP tools + built-in capabilities)
                tools = [
                    {
                        "function_declarations": functional_tools,
                        "code_execution": {},  # Enable code execution
                        "google_search": {},  # Enable web search
                    },
                ]

                # Configure Gemini Live session
                live_config = types.LiveConnectConfig(
                    response_modalities=[self.response_modality],
                    system_instruction=types.Content(
                        parts=[types.Part(text=system_instructions)]
                    ),
                    tools=tools,
                )

                try:
                    # Start Gemini Live session and create task group
                    async with (
                        self.client.aio.live.connect(
                            model=MODEL, config=live_config
                        ) as session,
                        asyncio.TaskGroup() as task_group,
                    ):
                        self.session = session
                        print("✅ Connected to Gemini Live")
                        print("🎯 Ready for robot control commands!")
                        print("=" * 50)

                        # Start all async tasks
                        send_text_task = task_group.create_task(self.send_text())
                        task_group.create_task(self.receive_responses())

                        # Wait for user to quit (send_text_task completes when user types 'q')
                        await send_text_task
                        raise asyncio.CancelledError("User requested exit")

                except asyncio.CancelledError:
                    # Normal exit when user types 'q'
                    print("\nGoodbye!")
                    pass
                except asyncio.ExceptionGroup as exception_group:
                    # Handle any errors that occurred in the task group
                    traceback.print_exception(exception_group)


def main():
    """Main entry point for the text-only client."""
    parser = argparse.ArgumentParser(
        description="Gemini Live Text-Only Client for ROS robot control"
    )
    parser.add_argument(
        "--responses",
        type=str,
        default=DEFAULT_RESPONSE_MODALITY,
        help="Response format from Gemini",
        choices=["TEXT", "AUDIO"],
    )
    args = parser.parse_args()

    # Initialize and run the client
    client = GeminiROSClient(response_modality=args.responses)
    asyncio.run(client.run())


if __name__ == "__main__":
    main()
