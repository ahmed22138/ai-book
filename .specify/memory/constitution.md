<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: All core principles have been redefined based on user input.
Added sections: Project Goal, API Endpoints, Constraints, Success Criteria explicitly defined.
Removed sections: Embodied Intelligence First, Comprehensive Curriculum Coverage, AI-Native Interactive Design, Original Content & No Plagiarism, Full-Stack Bonus Architecture, Technical Architecture Requirements, Content Standards, Development Workflow.
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics – RAG Chatbot (Connected to Docusaurus Book) Constitution

## Project Goal
A backend chatbot that answers ONLY from:
1) My Docusaurus book content (stored in Qdrant)
2) OR user-selected text (if provided)

## Core Principles

### I. Book Content as Single Source of Truth
Book content = single source of truth. No hallucinations. If answer not found → say “Not in the book.” Always cite chapter + section from the Docusaurus book.

### II. User-Selected Text Override
User-selected text overrides RAG search.

### III. Safe Robotics Guidance
Safe robotics guidance only.

### IV. Tech Stack Adherence
FastAPI backend, OpenAI embeddings + ChatGPT model, Qdrant vector DB (stores Docusaurus book), Neon Postgres for metadata/logs, ChatKit SDK for Docusaurus frontend UI.

### V. API Endpoints Definition
POST /ask → Answer from Docusaurus book (RAG); POST /ask/selected → Answer only from selected text; POST /ingest → Load Docusaurus book into Qdrant; GET /health.

## Constraints
- Fast responses (<3s)
- Max user text: 30,000 chars
- Context limit: 20k tokens

## Success Criteria
- Chatbot fully works inside Docusaurus
- Answer accuracy is based on Docusaurus book
- Zero hallucinations

## Governance

All development must conform to these principles. The constitution supersedes contradictory practices. Amendments require documentation and approval before implementation. Complexity decisions (e.g., technology choices) must reference this constitution's rationale.

**Version**: 1.1.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-07
