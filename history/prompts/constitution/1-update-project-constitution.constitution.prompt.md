---
id: 1
title: Update Project Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution","update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
null
---

## Prompt

```text
/sp.constitution

Project: Physical AI & Humanoid Robotics – RAG Chatbot (Connected to Docusaurus Book)

Goal:
A backend chatbot that answers ONLY from:
1) My Docusaurus book content (stored in Qdrant)
2) OR user-selected text (if provided)

Core Rules:
- Book content = single source of truth.
- No hallucinations. If answer not found → say “Not in the book.”
- Always cite chapter + section from the Docusaurus book.
- User-selected text overrides RAG search.
- Safe robotics guidance only.

Tech Stack:
- FastAPI backend
- OpenAI embeddings + ChatGPT model
- Qdrant vector DB (stores Docusaurus book)
- Neon Postgres for metadata/logs
- ChatKit SDK for Docusaurus frontend UI

API Endpoints:
- POST /ask → Answer from Docusaurus book (RAG)
- POST /ask/selected → Answer only from selected text
- POST /ingest → Load Docusaurus book into Qdrant
- GET /health

Constraints:
- Fast responses (<3s)
- Max user text: 30,000 chars
- Context limit: 20k tokens

Success:
- Chatbot fully works inside Docusaurus
- Answer accuracy is based on Docusaurus book
- Zero hallucinations
```

## Response snapshot

Updated the project constitution with the provided details, new version 1.1.0.

## Outcome

- ✅ Impact: Project constitution has been redefined and updated.
- 🧪 Tests: No tests were run as this was a documentation update.
- 📁 Files: One file was modified: .specify/memory/constitution.md
- 🔁 Next prompts: Continue with project development based on the updated constitution.
- 🧠 Reflection: The constitution was successfully updated based on user input, and a PHR was created using agent-native tools due to script unavailability.

## Evaluation notes (flywheel)

- Failure modes observed: create-phr.sh script not found.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure create-phr.sh script is available or improve agent-native PHR creation.
