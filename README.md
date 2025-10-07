# ROS MCP Client 🧠⇄🤖

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)
![Python](https://img.shields.io/badge/python-3.10%2B-blue)

![GitHub Repo stars](https://img.shields.io/github/stars/robotmcp/ros-mcp-server?style=social)
![GitHub last commit](https://img.shields.io/github/last-commit/robotmcp/ros-mcp-server)


<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/docs/images/framework.png"/>
</p>

The **ROS MCP Client** is a reference implementation of a Model Context Protocol (MCP) client, designed to connect directly with [ros-mcp-server](https://github.com/robotmcp/ros-mcp-server/).

Instead of using a Desktop LLM client, it acts as a bridge that integrates an LLM, enabling natural-language interaction with any ROS or ROS2 robot.

## 🧠 What It Does

`ros-mcp-client` implements the LLM-side of the MCP protocol.

It can:
- Connect to a `ros-mcp-server` over MCP (stdio or HTTP).
- Send natural language queries or structured requests to the robot without the need to integrate it with a Desktop LLM client
- Stream back feedback, sensor data, or responses from the server.
- Integrate with a local LLM (Gemini, Ollama, Nvidia NeMo).

In short, it lets you run an MCP-compatible client that speaks to robots via the MCP interface — useful for testing, local reasoning, or autonomous AI controllers.

---

## ⚙️ Key Features of the ROS MCP Client

- Implements MCP client specification — plug-and-play with the ROS MCP server.

- ROS-aware LLM interface — specialized prompts and handlers for robotics tasks.

- Supports bidirectional streaming — send commands, receive real-time topic feedback.

- LLM integration ready — use Gemini, Anthropic, or Ollama APIs as reasoning engines.

- Offline-capable — works entirely within local or LAN environments.

---

## 

## 🛠 Getting Started  

The MCP client is version-agnostic (ROS1 or ROS2).  

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/docs/images/MCP_topology.png"/>
</p>  

### Prerequisites

- ROS or ROS2 running with `rosbridge`
- Active [`ros-mcp-server`](https://github.com/robotmcp/ros-mcp-server) instance

### Installation  

1. Clone the repository  
```
git clone https://github.com/robotmcp/ros-mcp-client.git
cd ros-mcp-client
```

2. Install `uv` and `rosbridge`  

3. Prepare environment
```
uv sync # or pip install -e
```

4. Pick your model and run the client
```
cd gemini_client
uv run client.py
```

5. Start `rosbridge` on the target robot  

---

## 📚 Examples & Tutorials  

Browse our [examples](examples) to see the server in action. 
- `Google Gemini` client
- `Nvidia NeMo` client

We welcome community PRs with new examples and integrations!  

---

## 🤝 Contributing  

We love contributions of all kinds:  
- Bug fixes and documentation updates  
- New features (e.g., Action support, permissions)  
- Additional examples and tutorials  

Check out the [contributing guidelines](docs/contributing.md) and see issues tagged **good first issue** to get started.  

---

## 📜 License  

This project is licensed under the [Apache License 2.0](LICENSE).  