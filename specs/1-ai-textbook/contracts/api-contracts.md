# API Contracts: Physical AI & Humanoid Robotics Textbook

**Base URL**: `https://api.textbook.example.com` (or `http://localhost:8000` for local)
**API Version**: v1
**Documentation**: See OpenAPI spec below

---

## Authentication

All endpoints (except `/health` and `/docs`) require authentication via JWT Bearer token.

**Header**:
```
Authorization: Bearer <JWT_TOKEN>
```

**Token Acquisition**:
- **Signup**: `POST /auth/signup`
- **Signin**: `POST /auth/signin`
- **Refresh**: `POST /auth/refresh`

---

## Endpoints

### 1. RAG Chatbot Query

**Endpoint**: `POST /chat`

**Purpose**: Query the RAG chatbot for answers grounded in textbook content

**Request**:
```json
{
  "query": "Explain forward kinematics",
  "selected_text": "[Optional] The robot's position can be computed using...",
  "chapter": "[Optional] week-7-kinematics",
  "stream": false
}
```

**Request Schema**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| query | string | YES | 10-500 chars | User question or query |
| selected_text | string | NO | 50-5000 chars | Highlighted text from chapter |
| chapter | string | NO | Non-empty | Chapter context (e.g., "week-7-kinematics") |
| stream | boolean | NO | Default: false | Stream response token-by-token |

**Response (Success 200)**:
```json
{
  "id": "msg-550e8400-e29b-41d4-a716-446655440000",
  "query": "Explain forward kinematics",
  "response": "Forward kinematics is the process of calculating the end-effector position and orientation based on joint angles. Given joint angles θ1, θ2, ..., θn, we compute the position (x, y, z) and orientation (roll, pitch, yaw) using the Denavit-Hartenberg (DH) convention...",
  "sources": [
    {
      "chapter": "week-7-kinematics",
      "section": "3.1-forward-kinematics",
      "quote": "Forward kinematics computes the position and orientation of the end-effector given joint angles using the DH convention."
    },
    {
      "chapter": "week-8-motion-planning",
      "section": "2.2-dk-inverse-kinematics",
      "quote": "The inverse of forward kinematics is inverse kinematics, which solves for joint angles given a desired end-effector pose."
    }
  ],
  "confidence": 0.92,
  "response_time_ms": 1850,
  "feedback": null,
  "created_at": "2025-12-06T14:30:00Z"
}
```

**Response Schema**:
| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique message ID for tracking |
| query | string | Echo of original query |
| response | string | Generated answer |
| sources | array | Array of citation objects |
| sources[].chapter | string | Chapter identifier |
| sources[].section | string | Section within chapter |
| sources[].quote | string | Relevant excerpt from textbook |
| confidence | number | Confidence score (0.0-1.0) |
| response_time_ms | integer | Generation time in milliseconds |
| feedback | string \| null | User feedback: 'helpful', 'not_helpful', or null |
| created_at | string | ISO 8601 timestamp |

**Response (Error 400)**:
```json
{
  "error": "InvalidQueryError",
  "message": "Query must be between 10 and 500 characters",
  "status": 400,
  "timestamp": "2025-12-06T14:30:00Z"
}
```

**Response (Error 503)**:
```json
{
  "error": "ServiceUnavailable",
  "message": "Qdrant vector store is temporarily unavailable",
  "status": 503,
  "retry_after": 30
}
```

**Status Codes**:
| Code | Meaning | Cause |
|------|---------|-------|
| 200 | OK | Query processed successfully |
| 400 | Bad Request | Invalid query format or parameters |
| 401 | Unauthorized | Missing or invalid JWT token |
| 429 | Too Many Requests | Rate limit exceeded (100 req/min per user) |
| 503 | Service Unavailable | Qdrant or OpenAI API unavailable |

**Rate Limiting**:
- 100 requests per minute per user
- Header: `X-RateLimit-Remaining: 45`

**Caching**:
- Identical queries cached for 1 hour (redis)
- Cache key: `sha256(user_id + query + chapter)`

---

### 2. Submit Chatbot Feedback

**Endpoint**: `PUT /chat/{message_id}/feedback`

**Purpose**: Record user feedback on chatbot response quality

**Request**:
```json
{
  "feedback": "helpful"
}
```

**Request Schema**:
| Field | Type | Required | Constraints | Values |
|-------|------|----------|-------------|--------|
| feedback | string | YES | Non-empty | 'helpful', 'not_helpful', 'incorrect' |

**Response (Success 200)**:
```json
{
  "id": "msg-550e8400-e29b-41d4-a716-446655440000",
  "feedback": "helpful",
  "updated_at": "2025-12-06T14:35:00Z"
}
```

---

### 3. Translation API

**Endpoint**: `POST /translate`

**Purpose**: Translate a chapter to a target language (cached)

**Request**:
```json
{
  "chapter_id": "week-1-embodied-ai",
  "language": "ur"
}
```

**Request Schema**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| chapter_id | string | YES | Non-empty | Chapter identifier |
| language | string | YES | ISO 639-1 code | Target language (e.g., 'ur', 'es', 'fr') |
| force_refresh | boolean | NO | Default: false | Bypass cache and re-translate |

**Response (Success 200)**:
```json
{
  "chapter_id": "week-1-embodied-ai",
  "language": "ur",
  "content": "# جسمانی ہوشیاری کا تعارف\n\nیہ باب جسمانی ہوشیاری کے بنیادی اصولوں کا تعارف ہے...",
  "source_language": "en",
  "cached": false,
  "translation_model": "openai-gpt-4",
  "created_at": "2025-12-06T12:00:00Z",
  "expires_at": "2025-12-20T12:00:00Z"
}
```

**Response Schema**:
| Field | Type | Description |
|-------|------|-------------|
| chapter_id | string | Chapter identifier |
| language | string | Target language code |
| content | string | Translated markdown content |
| source_language | string | Original language (always 'en') |
| cached | boolean | Whether response was served from cache |
| translation_model | string | Model used for translation |
| created_at | string | ISO 8601 timestamp |
| expires_at | string | Cache expiration timestamp |

**Response (Error 404)**:
```json
{
  "error": "ChapterNotFound",
  "message": "Chapter 'week-1-embodied-ai' not found in textbook",
  "status": 404
}
```

**Response (Error 422)**:
```json
{
  "error": "UnsupportedLanguage",
  "message": "Language 'xyz' is not supported. Supported: ['ur', 'es', 'fr']",
  "status": 422,
  "supported_languages": ["ur", "es", "fr"]
}
```

**Status Codes**:
| Code | Meaning |
|------|---------|
| 200 | OK (translation retrieved or generated) |
| 400 | Bad Request (invalid chapter_id or language) |
| 401 | Unauthorized |
| 404 | Chapter not found |
| 422 | Unsupported language |
| 429 | Rate limit exceeded (50 req/min per user) |
| 503 | Service Unavailable |

**Rate Limiting**:
- 50 requests per minute per user (translations are expensive)

**Caching**:
- Cache duration: 14 days per language per chapter
- Cache key: `translation:{chapter_id}:{language}`

---

### 4. Subagent Invocation

**Endpoint**: `POST /agent/invoke`

**Purpose**: Call a reusable AI subagent (code generation, assessment creation, etc.)

**Request**:
```json
{
  "agent_name": "ros2-code-gen",
  "context": {
    "lesson": "week-9-real-time-control",
    "task": "motion-planning",
    "language": "python"
  },
  "stream": false
}
```

**Request Schema**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| agent_name | string | YES | Registered agent | Agent identifier |
| context | object | YES | Non-empty | Agent-specific input |
| stream | boolean | NO | Default: false | Stream output token-by-token |

**Valid Agents**:
| Agent Name | Purpose | Input Context | Output Type |
|------------|---------|---------------|------------|
| `ros2-code-gen` | Generate ROS 2 code | `{lesson, task, language}` | Python/C++ code |
| `assessment-gen` | Generate assessment questions | `{chapter, level, count}` | JSON array of questions |
| `diagram-gen` | Generate diagrams | `{concept, style}` | Mermaid/ASCII diagram |

**Response (Success 200)**:
```json
{
  "id": "agent-550e8400-e29b-41d4-a716-446655440001",
  "agent_name": "ros2-code-gen",
  "output": "#!/usr/bin/env python3\nimport rclpy\nfrom rclpy.node import Node\nfrom geometry_msgs.msg import Twist\n\nclass MotionPlannerNode(Node):\n    def __init__(self):\n        super().__init__('motion_planner')\n        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)\n        self.timer = self.create_timer(1.0, self.timer_callback)\n\n    def timer_callback(self):\n        msg = Twist()\n        msg.linear.x = 0.5\n        self.publisher.publish(msg)\n        self.get_logger().info('Publishing motion command')\n\nif __name__ == '__main__':\n    rclpy.init()\n    node = MotionPlannerNode()\n    rclpy.spin(node)\n",
  "execution_time_ms": 2500,
  "tokens_used": 1150,
  "cost_usd": 0.035,
  "created_at": "2025-12-06T15:20:00Z"
}
```

**Response Schema**:
| Field | Type | Description |
|-------|------|-------------|
| id | string | Unique invocation ID |
| agent_name | string | Agent that was invoked |
| output | string | Generated output |
| execution_time_ms | integer | Execution time in milliseconds |
| tokens_used | integer | LLM tokens consumed |
| cost_usd | number | Estimated API cost |
| created_at | string | ISO 8601 timestamp |

**Response (Error 400)**:
```json
{
  "error": "InvalidContextError",
  "message": "Agent 'ros2-code-gen' requires context keys: ['lesson', 'task', 'language']",
  "status": 400,
  "required_keys": ["lesson", "task", "language"]
}
```

**Response (Error 404)**:
```json
{
  "error": "AgentNotFound",
  "message": "Unknown agent: 'unknown-agent'",
  "status": 404,
  "available_agents": ["ros2-code-gen", "assessment-gen", "diagram-gen"]
}
```

**Status Codes**:
| Code | Meaning |
|------|---------|
| 200 | OK |
| 400 | Bad Request (invalid context) |
| 401 | Unauthorized |
| 404 | Agent not found |
| 429 | Rate limit exceeded (20 req/min per user - agents are expensive) |
| 503 | Service Unavailable |

**Rate Limiting**:
- 20 requests per minute per user
- Subagent calls are expensive; enforce strict rate limits

---

### 5. Authentication: Signup

**Endpoint**: `POST /auth/signup`

**Purpose**: Create a new user account with background questions

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePassword123!",
  "name": "Ahmed Khan",
  "software_experience": "Intermediate",
  "hardware_experience": true
}
```

**Request Schema**:
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| email | string | YES | Valid email | User email address |
| password | string | YES | ≥8 chars, mixed case/numbers | User password |
| name | string | YES | 2-100 chars | User's full name |
| software_experience | string | YES | 'Beginner','Intermediate','Expert' | Software background |
| hardware_experience | boolean | YES | true \| false | Robot experience |

**Response (Success 201)**:
```json
{
  "id": "user-550e8400-e29b-41d4-a716-446655440000",
  "email": "student@example.com",
  "name": "Ahmed Khan",
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600,
  "created_at": "2025-12-06T10:30:00Z"
}
```

**Response (Error 409)**:
```json
{
  "error": "EmailAlreadyExists",
  "message": "Email 'student@example.com' is already registered",
  "status": 409
}
```

**Status Codes**:
| Code | Meaning |
|------|---------|
| 201 | Created |
| 400 | Bad Request (validation error) |
| 409 | Conflict (email already exists) |

---

### 6. Authentication: Signin

**Endpoint**: `POST /auth/signin`

**Purpose**: Authenticate user and issue tokens

**Request**:
```json
{
  "email": "student@example.com",
  "password": "SecurePassword123!"
}
```

**Response (Success 200)**:
```json
{
  "id": "user-550e8400-e29b-41d4-a716-446655440000",
  "email": "student@example.com",
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "Bearer",
  "expires_in": 3600
}
```

**Status Codes**:
| Code | Meaning |
|------|---------|
| 200 | OK |
| 401 | Unauthorized (invalid credentials) |
| 404 | User not found |

---

### 7. User Profile

**Endpoint**: `GET /user/profile`

**Purpose**: Retrieve authenticated user's profile

**Request**:
```
GET /user/profile
Authorization: Bearer <JWT_TOKEN>
```

**Response (Success 200)**:
```json
{
  "id": "user-550e8400-e29b-41d4-a716-446655440000",
  "email": "student@example.com",
  "name": "Ahmed Khan",
  "profile": {
    "software_experience": "Intermediate",
    "hardware_experience": true,
    "personalization_level": "Intermediate",
    "preferred_language": "ur",
    "completed_chapters": ["week-1-embodied-ai", "week-2-robot-anatomy"]
  },
  "created_at": "2025-12-06T10:30:00Z"
}
```

---

### 8. Update Personalization

**Endpoint**: `PUT /user/profile/personalization`

**Purpose**: Update user's content complexity preference

**Request**:
```json
{
  "personalization_level": "Expert",
  "preferred_language": "en"
}
```

**Response (Success 200)**:
```json
{
  "personalization_level": "Expert",
  "preferred_language": "en",
  "updated_at": "2025-12-06T15:40:00Z"
}
```

---

### 9. Health Check

**Endpoint**: `GET /health`

**Purpose**: Service health status (no auth required)

**Response (Success 200)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-06T15:40:00Z",
  "services": {
    "database": "ok",
    "qdrant": "ok",
    "openai": "ok"
  }
}
```

**Response (Degraded 503)**:
```json
{
  "status": "degraded",
  "timestamp": "2025-12-06T15:40:00Z",
  "services": {
    "database": "ok",
    "qdrant": "error",
    "openai": "ok"
  },
  "message": "Qdrant service unreachable"
}
```

---

## Error Handling

### Standard Error Response

All error responses follow this format:

```json
{
  "error": "ErrorCode",
  "message": "Human-readable error description",
  "status": 400,
  "timestamp": "2025-12-06T15:40:00Z",
  "request_id": "req-550e8400-e29b-41d4-a716-446655440000"
}
```

### Common Error Codes

| Code | HTTP | Description |
|------|------|-------------|
| InvalidInput | 400 | Request validation failed |
| Unauthorized | 401 | Missing or invalid JWT token |
| Forbidden | 403 | User lacks permission |
| NotFound | 404 | Resource not found |
| Conflict | 409 | Resource already exists |
| RateLimited | 429 | Too many requests |
| ServiceError | 500 | Internal server error |
| ServiceUnavailable | 503 | Dependency (Qdrant, OpenAI) unavailable |

---

## Request/Response Examples (cURL)

### Chat Query

```bash
curl -X POST https://api.textbook.example.com/chat \
  -H "Authorization: Bearer <JWT>" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is forward kinematics?",
    "chapter": "week-7-kinematics"
  }'
```

### Translation

```bash
curl -X POST https://api.textbook.example.com/translate \
  -H "Authorization: Bearer <JWT>" \
  -H "Content-Type: application/json" \
  -d '{
    "chapter_id": "week-1-embodied-ai",
    "language": "ur"
  }'
```

### Agent Invocation

```bash
curl -X POST https://api.textbook.example.com/agent/invoke \
  -H "Authorization: Bearer <JWT>" \
  -H "Content-Type: application/json" \
  -d '{
    "agent_name": "ros2-code-gen",
    "context": {
      "lesson": "week-9-real-time-control",
      "task": "motion-planning",
      "language": "python"
    }
  }'
```

---

## OpenAPI Specification

Full OpenAPI 3.0.0 spec available at: `https://api.textbook.example.com/openapi.json`

Generated from FastAPI with:
```python
from fastapi.openapi.utils import get_openapi

app.openapi_schema = get_openapi(...)
```

