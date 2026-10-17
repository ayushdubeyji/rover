import json

with open(r'C:\Users\dell\.gemini\antigravity-cli\brain\44fae4d7-8f78-40ee-a614-f593c20efcb5\.system_generated\logs\transcript.jsonl', 'r', encoding='utf-8') as f:
    for i, line in enumerate(f):
        if 385 <= i <= 395:
            obj = json.loads(line)
            th = obj.get('thinking', '')
            if th:
                print(f"{i}: {th[:150]}")
            tc = obj.get('tool_calls')
            if tc:
                for c in tc:
                    print(f"   TOOL: {c.get('name')} {c.get('args')}")
