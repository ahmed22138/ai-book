# Data Model: Physical AI & Humanoid Robotics Textbook

## Entity Relationship Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                          ENTITIES                            │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌────────────────┐         ┌──────────────────┐           │
│  │    User        │◄───────►│  Profile         │           │
│  ├────────────────┤         ├──────────────────┤           │
│  │ id (PK)        │         │ user_id (FK)     │           │
│  │ email          │         │ software_exp     │           │
│  │ password_hash  │         │ hardware_exp     │           │
│  │ created_at     │         │ personalization  │           │
│  └────────────────┘         └──────────────────┘           │
│         ▲                                                    │
│         │ 1:N                                                │
│    ┌────┴───────────────────────────────────┐               │
│    │                                        │               │
│    ▼                                        ▼               │
│  ┌──────────────────┐          ┌─────────────────────┐     │
│  │  ChatMessage     │          │  Translation        │     │
│  ├──────────────────┤          ├─────────────────────┤     │
│  │ id (PK)          │          │ id (PK)             │     │
│  │ user_id (FK)     │          │ chapter_id          │     │
│  │ query            │          │ language            │     │
│  │ response         │          │ content             │     │
│  │ sources (JSON)   │          │ created_at          │     │
│  │ created_at       │          │ expires_at          │     │
│  └──────────────────┘          └─────────────────────┘     │
│                                                              │
│  ┌──────────────────────────────────────────┐              │
│  │  SubagentInvocation                      │              │
│  ├──────────────────────────────────────────┤              │
│  │ id (PK)                                  │              │
│  │ user_id (FK, optional - anon allowed)    │              │
│  │ agent_name                               │              │
│  │ input_payload (JSON)                     │              │
│  │ output                                   │              │
│  │ execution_time_ms                        │              │
│  │ created_at                               │              │
│  └──────────────────────────────────────────┘              │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

---

## Entity Definitions

### 1. User

**Purpose**: Represents authenticated learners accessing the textbook

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY | Unique user identifier |
| `email` | String(255) | UNIQUE, NOT NULL | User email, used for login |
| `password_hash` | String(255) | NOT NULL | Bcrypt/Argon2 hashed password |
| `name` | String(255) | NOT NULL | User's full name |
| `created_at` | Timestamp | NOT NULL, DEFAULT NOW() | Account creation time |
| `updated_at` | Timestamp | NOT NULL, DEFAULT NOW() | Last profile update |
| `last_login` | Timestamp | NULLABLE | Last authentication timestamp |

**Relationships**:
- 1:1 → Profile (user preferences & background)
- 1:N → ChatMessage (query history)
- 1:N → SubagentInvocation (agent usage tracking)

**Constraints**:
- Email must be valid format (enforced on signup)
- Password must be ≥8 characters (client-side) and hashed server-side
- Name must be non-empty

**Example**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "student@example.com",
  "name": "Ahmed Khan",
  "created_at": "2025-12-06T10:30:00Z",
  "last_login": "2025-12-06T15:45:30Z"
}
```

---

### 2. Profile (User Personalization)

**Purpose**: Stores user background, preferences, and personalization settings

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY | Profile record ID |
| `user_id` | UUID | FOREIGN KEY (User.id), UNIQUE | Link to user |
| `software_experience` | Enum | NOT NULL, DEFAULT 'Beginner' | Options: Beginner, Intermediate, Expert |
| `hardware_experience` | Boolean | NOT NULL, DEFAULT false | Has student built/used robots? |
| `personalization_level` | Enum | NOT NULL, DEFAULT 'Beginner' | Tracks current content complexity preference |
| `preferred_language` | String(10) | NOT NULL, DEFAULT 'en' | Language code (e.g., 'en', 'ur') |
| `completed_chapters` | Array(String) | NULLABLE, DEFAULT [] | List of chapter IDs completed |
| `created_at` | Timestamp | NOT NULL, DEFAULT NOW() | Profile creation time |
| `updated_at` | Timestamp | NOT NULL, DEFAULT NOW() | Last preference update |

**Relationships**:
- N:1 ← User

**Constraints**:
- Each user has exactly one profile (created at signup)
- `software_experience` must be one of predefined enum values
- `preferred_language` must be ISO 639-1 code

**Example**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440001",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "software_experience": "Intermediate",
  "hardware_experience": true,
  "personalization_level": "Intermediate",
  "preferred_language": "ur",
  "completed_chapters": ["week-1-embodied-ai", "week-2-robot-anatomy"],
  "updated_at": "2025-12-06T16:00:00Z"
}
```

---

### 3. ChatMessage

**Purpose**: Stores interactions between users and the RAG chatbot for history, analytics, and debugging

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY | Message record ID |
| `user_id` | UUID | FOREIGN KEY (User.id), NULLABLE | User ID (NULL for anonymous queries) |
| `query` | Text | NOT NULL | Original user question/query |
| `selected_text` | Text | NULLABLE | Text passage user highlighted (if applicable) |
| `chapter` | String(255) | NULLABLE | Chapter context where query originated |
| `response` | Text | NOT NULL | Chatbot's generated answer |
| `sources` | JSONB | NOT NULL, DEFAULT [] | Array of source citations |
| `confidence` | Float | NOT NULL, RANGE [0.0, 1.0] | Confidence score of RAG retrieval (0-1) |
| `response_time_ms` | Integer | NOT NULL | Time taken to generate response (milliseconds) |
| `tokens_used` | Integer | NULLABLE | LLM tokens consumed for this query |
| `feedback` | Enum | NULLABLE | User feedback: 'helpful', 'not_helpful', NULL | User satisfaction feedback |
| `created_at` | Timestamp | NOT NULL, DEFAULT NOW() | Query timestamp |

**Relationships**:
- N:1 ← User (optional)

**Constraints**:
- `query` and `response` must be non-empty
- `confidence` must be in range [0.0, 1.0]
- `response_time_ms` must be > 0
- Each `sources` array item must contain: `{chapter, section, quote}`

**Example**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440002",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "query": "Explain forward kinematics",
  "selected_text": "Forward kinematics computes the position and orientation...",
  "chapter": "week-7-kinematics",
  "response": "Forward kinematics is the process of calculating...",
  "sources": [
    {
      "chapter": "week-7-kinematics",
      "section": "3.1",
      "quote": "Forward kinematics is the mathematical process..."
    },
    {
      "chapter": "week-8-motion-planning",
      "section": "2.2",
      "quote": "Building on forward kinematics, we calculate..."
    }
  ],
  "confidence": 0.92,
  "response_time_ms": 1850,
  "tokens_used": 320,
  "feedback": "helpful",
  "created_at": "2025-12-06T14:30:00Z"
}
```

---

### 4. Translation (Cache)

**Purpose**: Stores translated content for efficient serving and caching

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY | Translation record ID |
| `chapter_id` | String(255) | NOT NULL, COMPOSITE KEY | Chapter identifier |
| `language` | String(10) | NOT NULL, COMPOSITE KEY | Target language (ISO 639-1: 'ur', 'es', etc.) |
| `content` | Text | NOT NULL | Translated chapter content (markdown format) |
| `source_language` | String(10) | NOT NULL, DEFAULT 'en' | Source language for translation |
| `translation_model` | String(50) | NOT NULL | Model used (e.g., 'openai-gpt-4') |
| `created_at` | Timestamp | NOT NULL, DEFAULT NOW() | Translation generation time |
| `expires_at` | Timestamp | NULLABLE | Cache expiration (optional, for refresh strategy) |
| `metadata` | JSONB | NULLABLE | Additional info (e.g., word count, char count) |

**Relationships**:
- No foreign keys (chapter_id is a reference, not enforced at DB level)

**Constraints**:
- `chapter_id` and `language` form a UNIQUE composite key
- `language` must be valid ISO 639-1 code
- `content` must be non-empty
- `expires_at`, if provided, must be > `created_at`

**Indexes**:
- UNIQUE on (`chapter_id`, `language`)
- INDEX on `expires_at` (for cache cleanup jobs)

**Example**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440003",
  "chapter_id": "week-1-embodied-ai",
  "language": "ur",
  "content": "# جسمانی ہوشیاری کا تعارف\n\nیہ باب...",
  "source_language": "en",
  "translation_model": "openai-gpt-4",
  "created_at": "2025-12-06T12:00:00Z",
  "expires_at": "2025-12-20T12:00:00Z",
  "metadata": {
    "source_word_count": 2500,
    "target_word_count": 2800,
    "api_cost_usd": 0.12
  }
}
```

---

### 5. SubagentInvocation

**Purpose**: Logs all AI agent invocations for analytics, debugging, and cost tracking

**Fields**:

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY | Invocation record ID |
| `user_id` | UUID | FOREIGN KEY (User.id), NULLABLE | User (NULL for anon demo) |
| `agent_name` | String(255) | NOT NULL | Agent identifier (e.g., 'ros2-code-gen') |
| `input_payload` | JSONB | NOT NULL | Input parameters passed to agent |
| `output` | Text | NOT NULL | Generated output/response |
| `execution_time_ms` | Integer | NOT NULL | Time to execute (milliseconds) |
| `tokens_input` | Integer | NULLABLE | Input tokens consumed |
| `tokens_output` | Integer | NULLABLE | Output tokens consumed |
| `cost_usd` | Float | NULLABLE | Estimated API cost for invocation |
| `status` | Enum | NOT NULL, DEFAULT 'success' | Options: 'success', 'error', 'timeout' |
| `error_message` | String(1000) | NULLABLE | Error details if status='error' |
| `created_at` | Timestamp | NOT NULL, DEFAULT NOW() | Invocation timestamp |

**Relationships**:
- N:1 ← User (optional)

**Constraints**:
- `agent_name` must match a known agent registry
- `execution_time_ms` must be ≥ 0
- `status` must be one of predefined enum values
- `tokens_input`, `tokens_output`, `cost_usd` should be populated for LLM-based agents

**Example**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440004",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "agent_name": "ros2-code-gen",
  "input_payload": {
    "lesson": "week-9-real-time-control",
    "task": "motion-planning",
    "language": "python"
  },
  "output": "#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\n...",
  "execution_time_ms": 2500,
  "tokens_input": 450,
  "tokens_output": 680,
  "cost_usd": 0.035,
  "status": "success",
  "created_at": "2025-12-06T15:20:00Z"
}
```

---

## Schema: SQL DDL

```sql
-- User table
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) NOT NULL UNIQUE,
  password_hash VARCHAR(255) NOT NULL,
  name VARCHAR(255) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP NOT NULL DEFAULT NOW(),
  last_login TIMESTAMP
);

-- User profiles (personalization)
CREATE TABLE profiles (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
  software_experience VARCHAR(50) NOT NULL DEFAULT 'Beginner' CHECK (software_experience IN ('Beginner', 'Intermediate', 'Expert')),
  hardware_experience BOOLEAN NOT NULL DEFAULT false,
  personalization_level VARCHAR(50) NOT NULL DEFAULT 'Beginner' CHECK (personalization_level IN ('Beginner', 'Intermediate', 'Expert')),
  preferred_language VARCHAR(10) NOT NULL DEFAULT 'en',
  completed_chapters TEXT[] DEFAULT ARRAY[]::TEXT[],
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Chat message history
CREATE TABLE chat_messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE SET NULL,
  query TEXT NOT NULL,
  selected_text TEXT,
  chapter VARCHAR(255),
  response TEXT NOT NULL,
  sources JSONB NOT NULL DEFAULT '[]'::jsonb,
  confidence DECIMAL(3, 2) NOT NULL CHECK (confidence >= 0 AND confidence <= 1),
  response_time_ms INTEGER NOT NULL,
  tokens_used INTEGER,
  feedback VARCHAR(50),
  created_at TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Translation cache
CREATE TABLE translations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(255) NOT NULL,
  language VARCHAR(10) NOT NULL,
  content TEXT NOT NULL,
  source_language VARCHAR(10) NOT NULL DEFAULT 'en',
  translation_model VARCHAR(50) NOT NULL,
  created_at TIMESTAMP NOT NULL DEFAULT NOW(),
  expires_at TIMESTAMP,
  metadata JSONB,
  UNIQUE (chapter_id, language)
);

-- Subagent invocation logs
CREATE TABLE subagent_invocations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE SET NULL,
  agent_name VARCHAR(255) NOT NULL,
  input_payload JSONB NOT NULL,
  output TEXT NOT NULL,
  execution_time_ms INTEGER NOT NULL,
  tokens_input INTEGER,
  tokens_output INTEGER,
  cost_usd DECIMAL(10, 4),
  status VARCHAR(50) NOT NULL DEFAULT 'success' CHECK (status IN ('success', 'error', 'timeout')),
  error_message VARCHAR(1000),
  created_at TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_profiles_user_id ON profiles(user_id);
CREATE INDEX idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX idx_chat_messages_chapter ON chat_messages(chapter);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at DESC);
CREATE INDEX idx_translations_chapter_lang ON translations(chapter_id, language);
CREATE INDEX idx_translations_expires_at ON translations(expires_at);
CREATE INDEX idx_subagent_invocations_user_id ON subagent_invocations(user_id);
CREATE INDEX idx_subagent_invocations_agent_name ON subagent_invocations(agent_name);
CREATE INDEX idx_subagent_invocations_created_at ON subagent_invocations(created_at DESC);
```

---

## State Transitions

### User Authentication State

```
[Unauthenticated] --signup--> [Email Verified] --login--> [Authenticated]
                                                  ^              |
                                                  |______________|
                                                    password reset
```

### Translation Cache State

```
[Requested] --translate-api--> [Generated] --cache-hit--> [Served]
                      |                            |
                      v                            v
                  [Error]                   [Expired/Refreshed]
```

---

## Validation Rules

| Entity | Field | Validation | Error Code |
|--------|-------|-----------|-----------|
| User | email | Valid email format | INVALID_EMAIL |
| User | password_hash | Non-empty, hashed | INVALID_PASSWORD |
| Profile | software_experience | Must be in enum | INVALID_EXPERIENCE_LEVEL |
| ChatMessage | response_time_ms | Must be > 0 | INVALID_RESPONSE_TIME |
| ChatMessage | confidence | Must be 0.0-1.0 | INVALID_CONFIDENCE |
| Translation | chapter_id | Non-empty string | INVALID_CHAPTER_ID |
| Translation | language | Valid ISO 639-1 code | INVALID_LANGUAGE_CODE |
| SubagentInvocation | agent_name | Must match registered agent | UNKNOWN_AGENT |
| SubagentInvocation | execution_time_ms | Must be ≥ 0 | INVALID_EXECUTION_TIME |

---

## Indexes & Performance

**Read-Heavy Queries** (optimized):
- `SELECT * FROM chat_messages WHERE user_id = ? ORDER BY created_at DESC` → index on `(user_id, created_at)`
- `SELECT * FROM translations WHERE chapter_id = ? AND language = ?` → unique composite index
- `SELECT * FROM subagent_invocations WHERE created_at > ? ORDER BY created_at DESC` → index on `created_at DESC`

**Write-Heavy Operations**:
- Chat message logging: bulk insert optimization (batch 100+ messages)
- Translation cache population: upsert pattern (INSERT ON CONFLICT DO UPDATE)
- Subagent logging: async batch writes

---

## Migration Strategy

**Phase 1 (Initial)**: Create all tables with indexes
**Phase 2 (Post-MVP)**: Add audit tables (for compliance) if needed
**Phase 3 (Optimization)**: Add materialized views for analytics

---

## Backup & Data Retention

- **User data**: Retain indefinitely (user has right to request deletion)
- **Chat messages**: Retain 1 year, then anonymize
- **Translation cache**: Retain until manual deletion or `expires_at` reached
- **Subagent logs**: Retain 90 days for analytics, then archive

