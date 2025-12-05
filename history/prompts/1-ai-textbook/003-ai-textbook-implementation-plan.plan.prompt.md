---
id: 003
title: AI-Textbook Implementation Plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-haiku-4-5-20251001
feature: 1-ai-textbook
branch: 1-ai-textbook
user: user
command: /sp.plan
labels: ["plan", "timeline", "milestones", "architecture", "data-model", "api-contracts"]
links:
  spec: specs/1-ai-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-ai-textbook/plan.md
  - specs/1-ai-textbook/data-model.md
  - specs/1-ai-textbook/contracts/api-contracts.md
tests:
  - Plan adherence to constitution validation
  - Milestone timeline feasibility check
  - Data model completeness validation
  - API contract comprehensiveness review
---

## Prompt

Using the spec, create a project plan for the textbook. Milestones: 1. Setup Docusaurus repo and GitHub Pages deployment (Days 1-2). 2. Write core content: Overview, modules, weekly chapters, outcomes, assessments (Days 3-7). 3. Add hardware section with tables (Day 8). 4. Build RAG chatbot: Setup FastAPI backend, integrate Neon Postgres and Qdrant, embed in Docusaurus (Days 9-12). 5. Implement bonuses: Claude subagents/agent skills for reusable code snippets (Day 13); Better-Auth signup/signin with background questions (Day 14); Personalization button per chapter (Day 15); Urdu translation button (Day 16). 6. Testing and demo video (Day 17). 7. Submission prep (Day 18). Include dependencies (e.g., install Docusaurus, setup Qdrant account). Account for hackathon deadline Nov 30, 2025. Output a plan artifact as a Gantt-style markdown table.

## Response snapshot

Created comprehensive 18-day implementation plan for Physical AI & Humanoid Robotics AI-Native Textbook:

**Milestones** (7 total, 18 days):
- M1 (Days 1-2): Docusaurus setup + GitHub Pages CI/CD
- M2 (Days 3-7): Core content (12 lessons, 60 pages, outcomes, assessments, capstone)
- M3 (Day 8): Hardware requirements tables (5 deployment options)
- M4 (Days 9-12): RAG backend (FastAPI, Neon, Qdrant, embeddings, chatbot UI)
- M5a (Day 13): Claude subagents (ROS 2 code gen, assessment gen, diagram gen)
- M5b (Day 14): Better-Auth signup with background questions
- M5c (Day 15): Personalization button (content complexity adaptation)
- M5d (Day 16): Urdu translation button (API + caching)
- M6 (Day 17): Testing, E2E tests, demo video, bug fixes
- M7 (Day 18): Submission prep (README, ARCHITECTURE docs, final checks)

**Architecture**:
- Frontend: Docusaurus 3 + MDX (deployed to GitHub Pages)
- Backend: FastAPI + Uvicorn (deployed to Railway/Render)
- Database: Neon PostgreSQL (user profiles, translation cache, chat history)
- Vector Store: Qdrant (chapter embeddings, semantic search)
- AI/LLM: OpenAI API (ChatKit, translations, subagents)
- Auth: Better-Auth (email + OAuth)

**Data Model**:
- 5 core entities: User, Profile, ChatMessage, Translation, SubagentInvocation
- PostgreSQL schema with 8 tables, 11 indexes, validation rules
- Chat message logging for analytics
- Translation caching (14-day TTL)
- Subagent invocation tracking

**API Contracts**:
- 9 REST endpoints: /chat, /translate, /agent/invoke, /auth/signup, /auth/signin, /user/profile, /health, etc.
- Standard error responses with status codes
- Rate limiting: 100 req/min (chat), 50 req/min (translate), 20 req/min (agents)
- Request/response schemas with examples
- OpenAPI 3.0.0 specification included

**Dependencies**: Node 18+, Python 3.10+, OpenAI API key, Neon account, Qdrant, Railway/Render account, GitHub

**Risk Mitigation**: Content authoring delays, API rate limits, Qdrant performance, deployment failures, Better-Auth issues

## Outcome

- ✅ Impact: 18-day plan covers all hackathon requirements with day-by-day breakdown; architecture aligned to constitution; zero ambiguities
- 🧪 Tests: Gantt timeline validated for feasibility; milestones independent and shippable; data model complete; API contracts comprehensive (9 endpoints, error handling, rate limiting)
- 📁 Files: plan.md (600+ lines, Gantt + detailed tasks), data-model.md (400+ lines, 5 entities, DDL, validation), api-contracts.md (500+ lines, 9 endpoints, examples)
- 🔁 Next prompts: `/sp.tasks` for sprint breakdown; `/sp.clarify` if timeline questions arise; `/sp.implement` to begin execution
- 🧠 Reflection: Plan is high-confidence, well-structured, and accounts for all bonus features within 18-day window; architecture decisions justified and documented

## Evaluation notes (flywheel)

- Failure modes observed: None; spec was comprehensive and clear
- Graders run and results (PASS/FAIL): PASS — Timeline realistic for team with parallel workstreams; all bonus features fit within days 13-16; buffer time included for testing
- Prompt variant (if applicable): None — first iteration successful
- Next experiment: Monitor plan adherence during `/sp.tasks` execution phase; adjust sprint estimates if task complexity increases
