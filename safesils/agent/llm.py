"""LLM backends for the agent.

The agent only needs "system + user text in, text out". Backends:
  none              no LLM: rule-based planning and root-cause notes
  ollama:<model>    local model through Ollama (default host http://localhost:11434,
                    override with OLLAMA_HOST), e.g. ollama:llama3.1
  openai:<model>    OpenAI API (needs OPENAI_API_KEY), e.g. openai:gpt-4o-mini
Tests use ScriptedLLM.
"""
from __future__ import annotations

import json
import os
import re
from collections.abc import Callable
from typing import Protocol


class LLM(Protocol):
    name: str

    def complete(self, system: str, user: str) -> str: ...


class LangChainLLM:
    def __init__(self, name: str, chat_model) -> None:
        self.name = name
        self.model = chat_model

    def complete(self, system: str, user: str) -> str:
        from langchain_core.messages import HumanMessage, SystemMessage

        reply = self.model.invoke([SystemMessage(content=system), HumanMessage(content=user)])
        return reply.content if isinstance(reply.content, str) else json.dumps(reply.content)


class ScriptedLLM:
    """Replays canned replies; a callable reply receives (system, user)."""

    def __init__(self, replies: list[str | Callable[[str, str], str]], name: str = "scripted") -> None:
        self.name = name
        self.replies = list(replies)
        self.calls: list[tuple[str, str]] = []

    def complete(self, system: str, user: str) -> str:
        self.calls.append((system, user))
        if not self.replies:
            return "{}"
        r = self.replies.pop(0)
        return r(system, user) if callable(r) else r


def make_llm(spec: str) -> LLM | None:
    kind, _, model = spec.partition(":")
    if kind == "none":
        return None
    if kind == "ollama":
        from langchain_ollama import ChatOllama

        host = os.environ.get("OLLAMA_HOST", "http://localhost:11434")
        if not host.startswith("http"):
            host = f"http://{host}"
        return LangChainLLM(spec, ChatOllama(model=model or "llama3.1", base_url=host, temperature=0, format="json"))
    if kind == "openai":
        from langchain_openai import ChatOpenAI

        return LangChainLLM(spec, ChatOpenAI(model=model or "gpt-4o-mini", temperature=0,
                                             model_kwargs={"response_format": {"type": "json_object"}}))
    raise ValueError(f"unknown LLM backend {spec!r}; use none, ollama:<model> or openai:<model>")


def parse_json(text: str) -> dict:
    """First JSON object in a reply, tolerating code fences and surrounding prose."""
    text = re.sub(r"^```(?:json)?|```$", "", text.strip(), flags=re.MULTILINE).strip()
    try:
        obj = json.loads(text)
    except json.JSONDecodeError:
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            raise ValueError("no JSON object in reply") from None
        obj = json.loads(text[start:end + 1])
    if not isinstance(obj, dict):
        raise ValueError("reply is not a JSON object")
    return obj
