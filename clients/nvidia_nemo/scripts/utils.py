# SPDX-FileCopyrightText: Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
from typing import Any

from pydantic import BaseModel, Field, create_model


def truncate_session_id(session_id: str, max_length: int = 10) -> str:
    """
    Truncate a session ID for logging purposes.

    Args:
        session_id: The session ID to truncate
        max_length: Maximum length before truncation (default: 10)

    Returns:
        Truncated session ID with "..." if longer than max_length, otherwise full ID
    """
    if len(session_id) > max_length:
        return session_id[:max_length] + "..."
    return session_id


def model_from_mcp_schema(name: str, mcp_input_schema: dict) -> type[BaseModel]:
    """
    Create a pydantic model from the input schema of the MCP tool
    """
    _type_map = {
        "string": str,
        "number": float,
        "integer": int,
        "boolean": bool,
        "array": list,
        "null": None,
        "object": dict,
    }

    properties = mcp_input_schema.get("properties", {})
    required_fields = set(mcp_input_schema.get("required", []))
    schema_dict = {}

    def _generate_valid_classname(class_name: str):
        return class_name.replace("_", " ").replace("-", " ").title().replace(" ", "")

    def _generate_field(field_name: str, field_properties: dict[str, Any]) -> tuple:
        json_type = field_properties.get("type", "string")
        enum_vals = field_properties.get("enum")

        if enum_vals:
            enum_name = f"{field_name.capitalize()}Enum"
            field_type = Enum(enum_name, {item: item for item in enum_vals})

        elif json_type == "object" and "properties" in field_properties:
            field_type = model_from_mcp_schema(
                name=field_name, mcp_input_schema=field_properties
            )
        elif json_type == "array" and "items" in field_properties:
            item_properties = field_properties.get("items", {})
            if item_properties.get("type") == "object":
                # CRITICAL FIX: Check if properties exist before creating nested model
                if "properties" in item_properties:
                    # Object with defined structure - create nested model
                    item_type = model_from_mcp_schema(
                        name=field_name, mcp_input_schema=item_properties
                    )
                else:
                    # Generic object without properties - use dict[str, Any]
                    item_type = dict[str, Any]
            else:
                item_type = _type_map.get(item_properties.get("type", "string"), Any)
            field_type = list[item_type]
        elif isinstance(json_type, list):
            # Union types (e.g., ["number", "null"])
            field_type = None
            has_null = "null" in json_type

            for t in json_type:
                if t == "null":
                    continue
                mapped = _type_map.get(t, Any)
                field_type = mapped if field_type is None else field_type | mapped

            # If field is not required and has null, make it truly optional
            if field_name not in required_fields and has_null:
                # Optional field - None is the default
                default_value = field_properties.get("default", None)
                if field_type:
                    field_type = field_type | None
            elif has_null:
                # Required but nullable
                default_value = field_properties.get("default", ...)
                if field_type:
                    field_type = field_type | None
            else:
                # No null in union - regular handling
                if field_name in required_fields:
                    default_value = field_properties.get("default", ...)
                else:
                    default_value = field_properties.get("default", None)

            return field_type, Field(
                default=default_value,
                description=field_properties.get("description", ""),
            )
        else:
            field_type = _type_map.get(json_type, Any)

        # Determine the default value based on whether the field is required
        if field_name in required_fields:
            # Field is required - use explicit default if provided, otherwise make it required
            default_value = field_properties.get("default", ...)
        else:
            # Field is optional - use explicit default if provided, otherwise None
            default_value = field_properties.get("default", None)
            # Make the type optional if no default was provided
            if "default" not in field_properties:
                field_type = field_type | None

        nullable = field_properties.get("nullable", False)
        description = field_properties.get("description", "")

        field_type = field_type | None if nullable else field_type

        return field_type, Field(default=default_value, description=description)

    for field_name, field_props in properties.items():
        schema_dict[field_name] = _generate_field(
            field_name=field_name, field_properties=field_props
        )

    return create_model(f"{_generate_valid_classname(name)}InputSchema", **schema_dict)
