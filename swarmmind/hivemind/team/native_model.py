"""SDK model-client extension for bounded OpenRouter/loopback tool arguments.

The installed llama.cpp build can ignore tool_choice. JSON-schema decoding is
reliable, so adapt one native tool's argument schema into a structured response.
The SDK still calls the model and owns all agent/tool execution. This adapter
never chooses a candidate, supplies a vote, or executes a tool itself.
"""

import json
from uuid import uuid4

from jsonschema import validate
from openjiuwen.core.common.clients import get_client_registry
from openjiuwen.core.foundation.llm import (
    AssistantMessageChunk,
    OpenAIModelClient,
    ToolCall,
    UserMessage,
)

from ..providers.openai_api import routing, tuning

PROVIDER = "SwarmMindStructured"


@get_client_registry().register_client(PROVIDER, "llm")
class LocalToolArguments(OpenAIModelClient):
    async def invoke(self, messages, *, tools=None, **kwargs):
        if len(tools or []) != 1:
            raise ValueError("native client requires exactly one authorized stage tool")
        tool = tools[0]
        name, schema = tool.name, tool.parameters
        prompt = UserMessage(content=f"Call {name}. Return only its JSON arguments matching this schema: "
                             + json.dumps(schema, separators=(",", ":")))
        # tools and response_format cannot both constrain llama.cpp decoding.
        # Reuse the SDK's HTTP client, accounting, error handling and timeouts.
        self.model_config.tool_choice = "none"
        # No tools are sent, so parallel_tool_calls is meaningless; no OpenRouter host
        # accepts it, and require_parameters would then reject every route.
        self.model_config.parallel_tool_calls = None
        model = self.model_config.model_name
        extra = routing(self.model_client_config.api_base + "/chat/completions", model)
        if extra:  # hosted only; a loopback server keeps the request it always had
            if not tuning(model)["temperature"]:
                self.model_config.temperature = None  # rejected by frontier hosts (M-88)
            kwargs["extra_body"] = {**(kwargs.get("extra_body") or {}), **extra}
        result = await super().invoke(
            [*messages, prompt], tools=None,
            response_format={"type": "json_schema", "json_schema": {
                "name": name, "strict": True, "schema": schema}}, **kwargs)
        if result.finish_reason != "stop" or not result.content:
            raise ValueError("Model did not complete its structured response")
        arguments = json.loads(result.content)
        validate(arguments, schema)  # fail closed even if the endpoint ignores its schema
        return result.model_copy(update={
            "content": "", "finish_reason": "tool_calls",
            "tool_calls": [ToolCall(id=f"rescue-{uuid4().hex}", type="function", name=name,
                                    arguments=json.dumps(arguments), index=0)],
        })

    async def stream(self, messages, *, tools=None, **kwargs):
        result = await self.invoke(messages, tools=tools, **kwargs)
        yield AssistantMessageChunk(**result.model_dump())
