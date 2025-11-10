# voice/llm_tools.py
from typing import TypedDict, Literal, Optional, List, Dict
import os, json, difflib
from dotenv import load_dotenv

load_dotenv()

try:
    from openai import OpenAI
except Exception:
    OpenAI = None  # allow offline

class Intent(TypedDict, total=False):
    action: Literal["go", "stop", "status", "ask", "none"]
    target: Optional[str]
    reason: str
    question: Optional[str]
    choices: Optional[List[str]]

# --- FAST LOCAL KEYWORD RULES ---
_RULES: Dict[str, str] = {
    # kitchen
    "hungry": "kitchen", "eat": "kitchen", "food": "kitchen",
    "cook": "kitchen", "wash dishes": "kitchen", "do the dishes": "kitchen",
    # bedroom
    "sleepy": "bedroom", "tired": "bedroom", "nap": "bedroom", "sleep": "bedroom",
    # living room
    "cozy": "living room", "tv": "living room", "sofa": "living room", "relax": "living room",
    # bathroom
    "toilet": "bathroom", "toilet paper": "bathroom", "restroom": "bathroom",
    "bathroom": "bathroom", "wash hands": "bathroom", "brush teeth": "bathroom",
}

_client_singleton = None
def get_client():
    """Lazy OpenAI client, returns None when offline/no key."""
    global _client_singleton
    if _client_singleton is not None:
        return _client_singleton
    api_key = os.getenv("OPENAI_API_KEY")
    if OpenAI is None or not api_key:
        return None
    _client_singleton = OpenAI(api_key=api_key)
    return _client_singleton

def _closest(s: str, options: List[str]) -> Optional[str]:
    m = difflib.get_close_matches(s, options, n=1, cutoff=0.0)
    return m[0] if m else None

def _local_rules(text: str, rooms: List[str]) -> Optional[str]:
    t = text.lower()
    for k, room in _RULES.items():
        if k in t and room in rooms:
            return room
    for r in rooms:
        if r in t:
            return r
    return None

_SYSTEM = (
    "You map user text to robot intents. "
    "Return ONLY JSON with keys: action, target, reason, question?, choices?. "
    "Allowed actions: go, stop, status, ask, none. "
    "target must be one of KNOWN_ROOMS or null. "
    "If ambiguous, use action='ask' with a short question and a choices array."
)

def parse_command(text: str, known_rooms: List[str]) -> Intent:
    t = text.lower().strip()

    # Normalize room names for case-insensitive matching
    rooms_map = {r.lower(): r for r in known_rooms}  # lc -> original
    rooms_lc = list(rooms_map.keys())

    # fast safety/status
    if any(w in t.split() for w in ["stop","halt","abort"]):
        return {"action":"stop","target":None,"reason":"safety word"}
    if "status" in t or "where am i" in t:
        return {"action":"status","target":None,"reason":"status word"}

    # offline rules & direct mentions
    room_lc = _local_rules(t, rooms_lc)
    if room_lc:
        return {"action": "go", "target": rooms_map[room_lc], "reason": "local rule / direct mention"}

    # vague “room” but not which one
    if "room" in t and not any(r in t for r in rooms_lc):
        return {"action":"ask","question":"Which room do you want to go to?",
                "choices": known_rooms, "reason":"ambiguous: room unspecified"}
    
    # NEW If user said "go …" but no known room was mentioned, ask directly (skip LLM)
    if (t.startswith(("go to", "go the", "go", "take me to", "bring me to"))
        and not any(r in t for r in rooms_lc)):
        return {
            "action": "ask",
            "question": "Which room do you want to go to?",
            "choices": known_rooms,
            "reason": "ambiguous: 'go …' without room",
        }

    # LLM fallback (optional)
    client = get_client()
    if client is None:
        return {"action":"ask","question":"Which room?", "choices":known_rooms,
                "reason":"no LLM / offline"}

    rooms_csv = ", ".join(sorted(known_rooms))
    user = f"KNOWN_ROOMS=[{rooms_csv}]\nUser said: {text!r}\nReturn JSON."

    try:
        resp = client.responses.create(
            model=os.getenv("LLM_MODEL", "gpt-4.1-mini"),
            input=[{"role":"system","content":_SYSTEM},
                   {"role":"user","content":user}],
            response_format={"type":"json_object"},
            temperature=0.2,
            # timeout=8
        )
        data = json.loads(resp.output[0].content[0].text)
        action = data.get("action","none")
        target = data.get("target")
        reason = data.get("reason","")
        question = data.get("question")
        choices = data.get("choices")

        if action not in {"go","stop","status","ask","none"}:
            return {"action":"none","target":None,"reason":"invalid action"}

        target_lc = (target or "").lower()

        if action == "go":
            # No target extracted at all → ask which room
            if not target_lc:
                return {
                    "action": "ask",
                    "question": "Which room do you want to go to?",
                    "choices": known_rooms,
                    "reason": "no target in utterance",
                }

            # Target not saved → ask, with a suggested guess if any
            if target_lc not in rooms_lc:
                guess_lc = _closest(target_lc, rooms_lc)
                if guess_lc:
                    guess = rooms_map[guess_lc]
                    return {
                        "action": "ask",
                        "question": f"I don't have '{target}' saved. Did you mean {guess}?",
                        "choices": [guess] + [r for r in known_rooms if r != guess],
                        "reason": "unknown room",
                    }
                return {
                    "action": "ask",
                    "question": f"I don't have '{target}' saved. Which room should I use?",
                    "choices": known_rooms,
                    "reason": "unknown room",
                }

            # Valid target → return original-cased room name
            return {"action": "go", "target": rooms_map[target_lc], "reason": reason}

        if action == "ask":
            if isinstance(choices, list):
                choices = [c for c in choices if c in known_rooms] or known_rooms
            else:
                choices = known_rooms
            return {"action":"ask","question":question or "Which room?",
                    "choices":choices, "reason":reason}

        if action in {"stop","status"}:
            return {"action":action,"target":None,"reason":reason}

        return {"action":"none","target":None,"reason":reason}
    except Exception as e:
        return {"action":"ask","question":"Which room?", "choices":known_rooms,
                "reason":f"api_error:{type(e).__name__}"}
