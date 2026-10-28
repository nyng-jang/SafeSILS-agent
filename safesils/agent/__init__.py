"""LLM verification agent on top of the deterministic SILS (step 4).

The agent turns a natural-language command into scenarios, runs them, explains
failures and searches for edge cases. PASS/FAIL always comes from safesils.judge.
"""
from .graph import VerificationAgent
from .llm import ScriptedLLM, make_llm

__all__ = ["VerificationAgent", "ScriptedLLM", "make_llm"]
