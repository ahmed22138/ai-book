# Implementation Plan: Physical AI & Humanoid Robotics AI-Native Textbook

**Feature Branch**: `1-ai-textbook`
**Created**: 2025-12-06
**Status**: Draft
**Hackathon Deadline**: 2025-11-30 (18 days from start)

---

## Executive Summary

This plan delivers a full-stack, AI-native textbook platform in 18 days across 7 milestones. The project prioritizes core learning content delivery (Days 1-8) followed by AI/backend integration (Days 9-12) and bonus features (Days 13-16), with testing and submission (Days 17-18). The architecture uses Docusaurus for the frontend, FastAPI for the backend, Neon PostgreSQL for storage, and Qdrant for semantic search.

---

## Timeline & Milestones

### Gantt Chart: Project Timeline

```
Day   Phase               Task                                          Duration  Status
────────────────────────────────────────────────────────────────────────────────────────
1-2   M1: Setup           Docusaurus repo, GitHub Pages config            2d       📋
      └─ [████████░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░]

3-7   M2: Core Content    Overview, modules, chapters, outcomes           5d       📋
      └─ [░░░░░░████████████████████░░░░░░░░░░░░░░░░░░░░░░░░░░░]

8     M3: Hardware        Hardware requirements tables & specs            1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░████░░░░░░░░░░░░░░░░░░░░░]

9-12  M4: RAG Backend     FastAPI, Neon, Qdrant, embedding              4d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████████████████░░░░░░]

13    M5a: Subagents      Claude agents for code generation             1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████░░░░]

14    M5b: Better-Auth    Signup/signin with background questions       1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████░░]

15    M5c: Personalize    Per-chapter content adaptation                1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████]

16    M5d: Translation    Urdu translation button & API                 1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████]

17    M6: Testing         E2E tests, demo video, bug fixes              1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████]

18    M7: Submission      Docs, README, final checks                    1d       📋
      └─ [░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░████]

────────────────────────────────────────────────────────────────────────────────────────
TOTAL: 18 days (2.5 weeks, sprint-based delivery)
```

---

## Milestone Details

### Milestone 1: Setup (Days 1-2) — Docusaurus & GitHub Pages

**Objectives**: Initialize project, configure GitHub Pages deployment, set up CI/CD

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Initialize Docusaurus 3 project | Dev | 0.5d | Node 18+ | `docusaurus.config.js` created, MDX enabled |
| Configure GitHub Pages deployment | Dev | 0.5d | Git repo, Docusaurus init | GitHub Actions workflow in `.github/workflows/` |
| Setup site structure (docs/, blog/, sidebar config) | Dev | 0.5d | Docusaurus init | Directories created, `sidebars.js` configured |
| Create homepage landing page | Content | 0.5d | Site structure | `index.md` with hero, CTA, overview |
| Deploy to GitHub Pages | Dev | 0.5d | GH Actions, site structure | Live site accessible at `https://[org].github.io/[repo]` |

**Deliverables**:
- Docusaurus site deployed to GitHub Pages
- CI/CD pipeline operational (auto-deploy on push)
- Homepage live with project branding

**Dependencies**:
- Node.js 18+
- Git & GitHub account
- Docusaurus documentation

---

### Milestone 2: Core Content (Days 3-7) — Curriculum & Lessons

**Objectives**: Write all learning content, define outcomes, create assessments

**Content Structure**:

```
docs/
  ├── 01-introduction/
  │   ├── week-1-embodied-ai-fundamentals.md
  │   ├── week-2-robot-anatomy-and-sensors.md
  │   ├── week-3-control-systems-basics.md
  │   └── assessments.md
  ├── 02-perception/
  │   ├── week-4-computer-vision-fundamentals.md
  │   ├── week-5-lidar-and-depth-sensing.md
  │   ├── week-6-sensor-fusion.md
  │   └── assessments.md
  ├── 03-control/
  │   ├── week-7-kinematics-and-inverse-kinematics.md
  │   ├── week-8-motion-planning.md
  │   ├── week-9-real-time-control.md
  │   └── assessments.md
  ├── 04-integration/
  │   ├── week-10-multi-robot-systems.md
  │   ├── week-11-human-robot-collaboration.md
  │   ├── week-12-deployment-and-scaling.md
  │   └── capstone-project.md
  └── reference/
      ├── learning-outcomes.md
      ├── assessment-rubrics.md
      └── glossary.md
```

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Write M1 (Intro) lessons (Weeks 1-3) | Content | 1.5d | Content outline | 3 chapters, ~15 pages, outcomes defined |
| Write M2 (Perception) lessons (Weeks 4-6) | Content | 1.5d | M1 complete | 3 chapters, ~15 pages, outcomes defined |
| Write M3 (Control) lessons (Weeks 7-9) | Content | 1.5d | M2 complete | 3 chapters, ~15 pages, outcomes defined |
| Write M4 (Integration) lessons (Weeks 10-12) | Content | 1.5d | M3 complete | 3 chapters, ~15 pages, outcomes defined |
| Create formative assessments (quizzes/prompts) | QA | 1d | All lessons | 30+ assessment items across modules |
| Create summative assessments (module exams) | QA | 1d | Formative assessments | 4 module exams, rubrics defined |
| Define capstone project with rubric | Arch | 1d | All modules | Capstone spec, success criteria, grading rubric |

**Deliverables**:
- 12 weekly lessons (~60 pages total)
- Learning outcomes defined per chapter (Bloom's taxonomy levels)
- Formative and summative assessments complete
- Capstone project specification with grading rubric

**Dependencies**:
- Content outline from spec
- Docusaurus site structure

---

### Milestone 3: Hardware (Day 8) — Specifications & Requirements

**Objectives**: Define hardware options, create comparison tables, link to lessons

**Hardware Options**:

| Option | Use Case | Cost Range | Key Components | Lessons |
|--------|----------|-----------|-----------------|---------|
| **Workstation (GPU)** | Development, simulation | $1k-$5k | RTX GPU, 16GB+ RAM, Ubuntu 22.04 | All |
| **Edge Kit (Jetson)** | Embedded perception, edge inference | $200-$800 | NVIDIA Jetson Orin, 12GB VRAM, IMX camera | M2, M3, M4 |
| **Physical Robot** | Embodied learning, hardware control | $5k-$50k+ | Robot arm/mobile base (e.g., Spot, custom), lidar, cameras | M1-M4 |
| **Cloud/Simulation** | No hardware access | Free-$100/mo | Gazebo Sim, AWS RoboMaker, GCP | M1-M4 |
| **Economy Kit** | Budget learning, hobby | $100-$500 | Arduino, servos, sensors, chassis | M1, M2, M3 |

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Define hardware requirements per module | Arch | 0.5d | Spec, lessons | Mapping of hardware to 12 lessons |
| Create hardware comparison table | Content | 0.3d | Hardware specs | Markdown table with 5 options, costs, use cases |
| Document setup guides per hardware option | Tech | 0.2d | Hardware specs | 5 setup guides (OS, drivers, dependencies) |

**Deliverables**:
- Hardware requirements table (5 options)
- Setup guides for each hardware platform
- Links from lessons to applicable hardware options

**Dependencies**:
- Core content complete
- Hardware research

---

### Milestone 4: RAG Chatbot & Backend (Days 9-12) — FastAPI, Neon, Qdrant

**Objectives**: Build backend services, integrate knowledge base, embed in Docusaurus

**Architecture**:

```
Frontend (Docusaurus)
    ↓ (HTTP/REST)
FastAPI Backend
    ├─ /chat (query chatbot)
    ├─ /translate (Urdu translation)
    ├─ /agent/invoke (subagent calls)
    └─ /health (status)
    ↓
Qdrant Vector Store ────→ Chapter embeddings
    ↓
Neon PostgreSQL ────────→ User profiles, translations cache
    ↓
OpenAI API ────────────→ ChatKit, translations, agents
```

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Setup FastAPI project & scaffold | Dev | 0.5d | Python 3.10+ | `main.py`, virtual env, requirements.txt |
| Integrate Neon PostgreSQL | Dev | 0.5d | FastAPI, Neon account | Connection pool, schema created (users, translations) |
| Setup Qdrant (local or cloud) | Dev | 0.5d | Qdrant account | Qdrant instance accessible, collection created |
| Generate embeddings for chapter content | Dev | 1d | All chapters, OpenAI key | Embeddings stored in Qdrant (~12 chapters × 50KB each) |
| Implement `/chat` endpoint with RAG | Dev | 1d | Qdrant, OpenAI, FastAPI | Endpoint returns answers with source citations |
| Implement text selection query support | Dev | 0.5d | /chat endpoint | Selected text forwarded to RAG pipeline |
| Create Docusaurus chat component (MDX) | Frontend | 0.5d | /chat endpoint live | Chat UI embedded in chapters, queries working |
| Setup error handling & retry logic | Dev | 0.5d | All endpoints | Graceful degradation, timeout handling |

**Deliverables**:
- FastAPI backend running on cloud (Railway, Render, or similar)
- Neon PostgreSQL provisioned and connected
- Qdrant vector store populated with embeddings
- `/chat` endpoint fully functional with streaming responses
- Docusaurus chat component integrated and tested

**Dependencies**:
- All chapters written (content to embed)
- OpenAI API key
- Neon account, Qdrant account
- Python 3.10+

---

### Milestone 5a: Reusable Subagents (Day 13) — Claude AI Agents

**Objectives**: Implement 3+ modular AI agents for code generation, assessments, diagrams

**Agents**:

| Agent | Purpose | Input | Output | Reusable Contexts |
|-------|---------|-------|--------|-------------------|
| **ROS 2 Code Gen** | Generate ROS 2 boilerplate for lessons | `{context, lesson, task}` | Python/C++ code, comments | M1, M3, M4 |
| **Assessment Gen** | Generate quiz/exam questions | `{chapter, level, count}` | Questions with rubrics | All modules |
| **Diagram Gen** | Create ASCII/SVG diagrams for concepts | `{concept, style}` | Markdown diagram + description | M1-M4 |

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Design agent contract & input/output schema | Arch | 0.3d | Spec | OpenAPI spec for `/agent/invoke` |
| Implement ROS 2 code generation agent | Dev | 0.4d | FastAPI, Claude API | Agent endpoint returns working code |
| Implement assessment generation agent | Dev | 0.3d | FastAPI, Claude API | Agent generates 5+ questions with rubrics |
| Implement diagram generation agent | Dev | 0.3d | FastAPI, Claude API | Agent returns Mermaid/ASCII diagrams |
| Document agent usage in chapters | Content | 0.3d | All agents working | 3+ chapters reference and use agents |
| Add agent invocation logging | Dev | 0.3d | All agents | Usage analytics available for review |

**Deliverables**:
- `/agent/invoke` endpoint operational with 3+ agents
- Agent contracts documented
- Agents callable from frontend buttons in chapters
- Logging and monitoring setup

**Dependencies**:
- FastAPI backend running
- Claude API access

---

### Milestone 5b: Authentication & Profile (Day 14) — Better-Auth

**Objectives**: Implement signup/signin, collect background, enable personalization

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Setup Better-Auth (email provider) | Dev | 0.4d | Neon PostgreSQL, FastAPI | Auth schema in Neon, token generation working |
| Create signup form with background questions | Frontend | 0.3d | Better-Auth | Form collects: name, email, password, software exp, hardware exp |
| Implement profile storage & retrieval | Dev | 0.3d | Neon, Better-Auth | User profiles persisted, retrievable via API |
| Add OAuth providers (optional: GitHub, Google) | Dev | 0.3d | Better-Auth, OAuth apps | OAuth signup alternative available |
| Implement session management | Dev | 0.3d | Better-Auth, Docusaurus | Sessions persist across page reloads |
| Create user dashboard (profile view/edit) | Frontend | 0.4d | Profile storage, session | Dashboard shows profile, background, personalization settings |

**Deliverables**:
- Better-Auth fully integrated
- Signup flow with background questions
- User profiles stored and retrievable
- Session management working
- User dashboard created

**Dependencies**:
- Neon PostgreSQL
- FastAPI backend
- Docusaurus frontend

---

### Milestone 5c: Personalization (Day 15) — Content Adaptation

**Objectives**: Implement "Personalize" button, adapt content complexity based on user background

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Define personalization schema (complexity levels) | Arch | 0.3d | User profiles | Content marked as Beginner/Intermediate/Expert |
| Annotate chapters with complexity levels | Content | 0.5d | All chapters | Each lesson section tagged with 1-3 complexity levels |
| Implement personalization endpoint | Dev | 0.3d | Complexity schema, profiles | `/personalize` returns filtered content per user level |
| Create "Personalize" button component | Frontend | 0.3d | Personalization endpoint | Button toggles content complexity, state persisted |
| Add filtering logic (show/hide by level) | Frontend | 0.3d | Button component | Content filtered client-side and server-side |

**Deliverables**:
- Content annotated with complexity levels
- "Personalize" button functional on all chapters
- Content adapts to user background (Beginner shows simplified, Expert shows advanced)
- Personalization preference persisted

**Dependencies**:
- User authentication (Day 14)
- All chapters written

---

### Milestone 5d: Urdu Translation (Day 16) — Multi-Language Support

**Objectives**: Implement "Translate to Urdu" button, cache translations, support language switching

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Setup OpenAI translation API integration | Dev | 0.3d | OpenAI key, FastAPI | `/translate` endpoint accepts chapter, language |
| Implement translation caching in Neon | Dev | 0.3d | Neon, FastAPI | Cache table created, lookups working |
| Create "Translate" button component | Frontend | 0.3d | Translation endpoint | Button triggers translation, shows Urdu content |
| Add language toggle UI | Frontend | 0.3d | Translate button | Toggle between original and translated versions |
| Batch-translate all chapters (optional background job) | Dev | 0.5d | Translation endpoint | All chapters pre-translated for faster serving |
| Add language selector to navbar | Frontend | 0.2d | Language toggle | Language preference persisted per user |

**Deliverables**:
- "/translate" endpoint functional
- "Translate to Urdu" button on all chapters
- Translations cached in Neon
- Language switching working (English ↔ Urdu)
- <5 second first-request translation latency

**Dependencies**:
- OpenAI API
- Neon PostgreSQL
- Docusaurus frontend

---

### Milestone 6: Testing & Demo (Day 17) — QA & Presentation

**Objectives**: Run E2E tests, record demo video, fix bugs, performance testing

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Write E2E tests (Playwright/Cypress) | QA | 0.4d | All features complete | 20+ test cases covering happy paths & edge cases |
| Run performance testing | QA | 0.3d | All features | Site <3s TTFB, /chat <2s response, translation <5s |
| Test cross-browser compatibility | QA | 0.3d | All features | Chrome, Firefox, Safari all pass |
| Record demo video (5-10 min) | Marketing | 0.4d | All features | Demo covers: homepage, lesson, chatbot, translation, personalization, auth |
| Fix critical bugs | Dev | 0.3d | Test results | Zero P1/P2 bugs, P3 bugs logged for future sprints |
| Create user acceptance test (UAT) checklist | QA | 0.2d | Spec, features | Checklist covers all user stories |

**Deliverables**:
- E2E test suite with 20+ passing tests
- Performance metrics recorded
- Demo video (5-10 minutes)
- All critical bugs fixed
- UAT checklist passed

**Dependencies**:
- All milestones 1-5 complete

---

### Milestone 7: Submission Prep (Day 18) — Documentation & Release

**Objectives**: Finalize documentation, README, deployment checklist, prepare submission

**Tasks**:

| Task | Owner | Duration | Dependencies | Definition of Done |
|------|-------|----------|--------------|-------------------|
| Write comprehensive README.md | Tech/Marketing | 0.3d | All features | Installation, setup, usage, architecture overview |
| Create ARCHITECTURE.md (design decisions) | Arch | 0.3d | Plan complete | Technical architecture, data models, API contracts |
| Document environment variables & secrets | Tech | 0.2d | All services | `.env.example` created, all secrets documented |
| Create deployment checklist | Ops | 0.2d | All services | Checklist for GitHub Pages, FastAPI, Neon, Qdrant |
| Verify GitHub Pages site live & accessible | Ops | 0.2d | Deployment checklist | Live URL works, site loads <3s, no 404s |
| Create submission artifacts (ZIP, links) | Marketing | 0.2d | All above | Repo link, live site link, demo video link ready |
| Final code review & merge to main | Lead | 0.2d | All features tested | Feature branch merged, main branch deployment ready |

**Deliverables**:
- README.md with full setup instructions
- ARCHITECTURE.md documenting design decisions
- Deployment checklist
- Live site deployed and verified
- Submission package ready (links, video, repo)

**Dependencies**:
- All milestones 1-6 complete

---

## Technical Context & Architecture

### Technology Stack Decision

| Layer | Technology | Rationale | Alternatives Considered |
|-------|-----------|-----------|------------------------|
| **Frontend** | Docusaurus 3 + MDX | Static site generation + interactive components; GitHub Pages native support | Next.js, Astro |
| **Backend** | FastAPI (Python) | Async REST API, OpenAI SDK integration, easy deployment | Flask, Django |
| **Auth** | Better-Auth | Lightweight, Neon-native, email + OAuth support | Auth0, Supabase Auth |
| **Database** | Neon PostgreSQL | Serverless, free tier sufficient, JSONB support for user profiles | Firebase, MongoDB |
| **Vector Store** | Qdrant | Open-source, easy self-host + managed option, high performance | Pinecone, Weaviate |
| **AI/LLM** | OpenAI API (ChatKit) | Latest models, reliable API, strong RAG support | Claude API, Anthropic |
| **Deployment** | GitHub Pages (frontend) + Railway/Render (backend) | Free/cheap tier, auto-deploy, minimal DevOps | Vercel, Heroku, AWS |

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      FRONTEND (Docusaurus)                  │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ • Homepage                                           │   │
│  │ • 12 Lesson Chapters (Module 1-4)                    │   │
│  │ • Chat Component (RAG Q&A)                           │   │
│  │ • Personalize Button (Complexity Toggle)             │   │
│  │ • Translate Button (Urdu)                            │   │
│  │ • Auth UI (Signup/Signin)                            │   │
│  └──────────────────────────────────────────────────────┘   │
│         Deployed: GitHub Pages                              │
└─────────────────────────────────────────────────────────────┘
                          ↓ HTTP/REST
┌─────────────────────────────────────────────────────────────┐
│                    BACKEND (FastAPI)                        │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ • POST /chat (RAG queries)                           │   │
│  │ • POST /translate (Urdu translation + caching)       │   │
│  │ • POST /agent/invoke (Subagent calls)                │   │
│  │ • GET /health (status check)                         │   │
│  │ • Auth endpoints (signup, signin, token refresh)     │   │
│  └──────────────────────────────────────────────────────┘   │
│         Deployed: Railway/Render                            │
└─────────────────────────────────────────────────────────────┘
    ↓                    ↓                    ↓
┌─────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│  Neon Postgres  │ │  Qdrant Vectors  │ │  OpenAI API      │
│  • Users        │ │  • Embeddings    │ │  • ChatKit       │
│  • Profiles     │ │  • Similarity    │ │  • Translation   │
│  • Translations │ │    Search        │ │  • Agents        │
│  • Cache        │ │  • Top-K Queries │ │                  │
└─────────────────┘ └──────────────────┘ └──────────────────┘
```

### Data Model (Preview)

**User**
- `id` (UUID)
- `email` (string, unique)
- `password_hash` (string)
- `software_experience` (enum: Beginner/Intermediate/Expert)
- `hardware_experience` (boolean)
- `personalization_level` (enum: Beginner/Intermediate/Expert)
- `created_at` (timestamp)
- `updated_at` (timestamp)

**Translation (Cache)**
- `id` (UUID)
- `chapter_id` (string)
- `language` (string, e.g., "ur" for Urdu)
- `content` (text)
- `translated_at` (timestamp)
- `expires_at` (timestamp, for cache invalidation)

**ChatMessage**
- `id` (UUID)
- `user_id` (UUID, foreign key)
- `query` (text)
- `selected_text` (text, optional)
- `response` (text)
- `sources` (JSON array of citations)
- `created_at` (timestamp)

**SubagentInvocation**
- `id` (UUID)
- `agent_name` (string)
- `input_payload` (JSON)
- `output` (text)
- `execution_time_ms` (integer)
- `created_at` (timestamp)

### API Contracts (Preview)

**POST /chat** (RAG Chatbot)
```json
Request: {
  "query": "Explain forward kinematics",
  "selected_text": "...",
  "chapter": "week-7-kinematics"
}

Response: {
  "answer": "Forward kinematics is...",
  "sources": [
    {"chapter": "week-7", "section": "3.1", "quote": "..."},
    {"chapter": "week-8", "section": "2.2", "quote": "..."}
  ],
  "confidence": 0.92
}
```

**POST /translate**
```json
Request: {
  "chapter_id": "week-1-embodied-ai",
  "language": "ur"
}

Response: {
  "chapter_id": "week-1-embodied-ai",
  "language": "ur",
  "content": "تعمیر شدہ ذہانت..."
}
```

**POST /agent/invoke**
```json
Request: {
  "agent_name": "ros2-code-gen",
  "context": {"lesson": "week-9", "task": "motion-planning"}
}

Response: {
  "agent_name": "ros2-code-gen",
  "output": "#!/usr/bin/env python3\nimport rclpy\n...",
  "execution_time_ms": 2500
}
```

---

## Constitution Check

Aligns with project constitution ✅:

- **I. Embodied Intelligence First**: Plan prioritizes content linking theory to physical robots (hardware spec tables, module structure)
- **II. Comprehensive Curriculum Coverage**: 12 weeks across 4 modules with outcomes, assessments, capstone (Milestone 2, 3)
- **III. AI-Native Interactive Design**: MDX components, RAG chatbot, personalization, translation, agents (Milestones 4, 5)
- **IV. Original Content & No Plagiarism**: Content authoring from scratch (Milestone 2); educators validate accuracy
- **V. Full-Stack Bonus Architecture**: All bonuses implemented (subagents, Better-Auth, personalization, Urdu translation) by Day 16
- **VI. RAG Chatbot & Knowledge Search**: FastAPI + Qdrant + OpenAI semantic search (Milestone 4)

---

## Dependencies & Prerequisites

### External Services Required

- **OpenAI API**: ChatKit, translation, agent inference (budget: ~$100-200 for hackathon)
- **Neon PostgreSQL**: Free tier (up to 3 projects, 10 GB storage)
- **Qdrant Cloud**: Free tier or self-host (managed free tier available)
- **Railway/Render**: Free tier for FastAPI backend deployment
- **GitHub Pages**: Free for public repos

### Tools & Software Required

| Tool | Version | Purpose |
|------|---------|---------|
| Node.js | 18+ | Docusaurus, build system |
| Python | 3.10+ | FastAPI, backend development |
| Git | 2.30+ | Version control |
| Docker (optional) | 24+ | Local FastAPI/Qdrant development |
| Playwright/Cypress | Latest | E2E testing |

### Development Environment Setup (Day 0.5)

```bash
# Frontend
npm install -g docusaurus/cli
npm init docusaurus my-textbook classic
npm install @docusaurus/plugin-ideal-image

# Backend
python -m venv venv
pip install fastapi uvicorn openai qdrant-client psycopg[binary] better-auth

# Databases (optional - use managed services for hackathon)
# docker run -p 6333:6333 qdrant/qdrant
```

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Content authoring delays | Medium | High | Pre-write outline, use AI for drafting assistance |
| API rate limits (OpenAI) | Medium | Medium | Implement caching, batch requests, fallback to simpler LLM |
| Qdrant embedding performance | Low | High | Use pre-computed embeddings, test with sample data early |
| GitHub Pages deployment fails | Low | High | Test CI/CD pipeline on Day 2, have manual deployment option |
| Better-Auth integration issues | Low | Medium | Setup early on Day 14, use managed auth if needed |

---

## Success Metrics

### MVP Completion (Days 1-12)

- ✅ Docusaurus site deployed to GitHub Pages (Day 2)
- ✅ 12 lessons published with outcomes & assessments (Day 7)
- ✅ Hardware requirements table complete (Day 8)
- ✅ RAG chatbot answering questions (Day 12)

### Full Feature Delivery (Days 13-16)

- ✅ 3+ subagents operational
- ✅ Better-Auth signup with background questions
- ✅ Personalize button adapting content
- ✅ Urdu translation button working

### Quality Gates (Day 17)

- ✅ 20+ E2E tests passing
- ✅ <3s GitHub Pages TTFB
- ✅ <2s chatbot response
- ✅ <5s translation latency
- ✅ 95% cross-browser compatibility

### Submission Ready (Day 18)

- ✅ README, ARCHITECTURE docs complete
- ✅ Demo video recorded (5-10 min)
- ✅ Live site accessible and fully functional
- ✅ All bonus features verified working

---

## Phase 0 Research Tasks (To Resolve)

All technical decisions in the spec are well-defined and aligned with constitution. No NEEDS CLARIFICATION markers identified. The following are confirmatory research tasks:

- **Best practices for RAG with Qdrant**: How to structure embeddings for optimal retrieval (complete before Day 9)
- **OpenAI ChatKit pricing model**: Confirm budget for 18-day sprint ($100-200 estimate) (complete by Day 5)
- **Better-Auth Neon integration**: Confirm schema compatibility and OAuth provider setup (complete by Day 13)
- **Docusaurus MDX performance**: Verify MDX components work efficiently with heavy interactive content (test on Day 2)

---

## Next Steps

1. **Day 0.5**: Setup development environment, confirm all accounts (Neon, Qdrant, OpenAI, Railway/Render)
2. **Day 1**: Start Milestone 1 (Docusaurus setup & deployment)
3. **Daily**: Update progress tracking, address blockers same-day
4. **Day 7**: Conduct mid-point review (content complete, RAG planning underway)
5. **Day 12**: RAG backend fully functional, begin bonus features
6. **Day 16**: All features complete, begin testing
7. **Day 18**: Submit! 🚀

---

**Plan Status**: ✅ Ready for Implementation

**Next Command**: `/sp.tasks` — Generate actionable task breakdown with dependencies and estimated story points

