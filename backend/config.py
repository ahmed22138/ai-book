"""
Configuration management for FastAPI backend
"""
import os
from dotenv import load_dotenv
from pydantic_settings import BaseSettings

# Load environment variables from .env file
load_dotenv()


class Settings(BaseSettings):
    """Application settings from environment variables"""

    # App
    app_name: str = "Physical AI Textbook API"
    app_version: str = "1.0.0"
    environment: str = os.getenv("ENVIRONMENT", "development")
    debug: bool = os.getenv("DEBUG", "false").lower() == "true"
    log_level: str = os.getenv("LOG_LEVEL", "info")

    # Database (Neon PostgreSQL)
    database_url: str = os.getenv("DATABASE_URL", "")
    db_host: str = os.getenv("DB_HOST", "localhost")
    db_port: int = int(os.getenv("DB_PORT", "5432"))
    db_name: str = os.getenv("DB_NAME", "textbook")
    db_user: str = os.getenv("DB_USER", "")
    db_password: str = os.getenv("DB_PASSWORD", "")

    # Qdrant Vector Store
    qdrant_url: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    qdrant_collection_name: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook_embeddings")

    # OpenAI
    openai_api_key: str = os.getenv("OPENAI_API_KEY", "")
    openai_org_id: str = os.getenv("OPENAI_ORG_ID", "")
    openai_model: str = os.getenv("OPENAI_MODEL", "gpt-4")
    openai_embedding_model: str = os.getenv("OPENAI_EMBEDDING_MODEL", "text-embedding-3-small")

    # Authentication
    auth_secret: str = os.getenv("AUTH_SECRET", "your-secret-key-change-in-production")
    jwt_secret: str = os.getenv("JWT_SECRET", "your-jwt-secret-change-in-production")
    jwt_expiry: int = int(os.getenv("JWT_EXPIRY", "3600"))
    refresh_token_expiry: int = int(os.getenv("REFRESH_TOKEN_EXPIRY", "604800"))

    # CORS
    cors_origins: list = ["http://localhost:3000", "https://your-username.github.io"]
    cors_credentials: bool = True
    cors_methods: list = ["*"]
    cors_headers: list = ["*"]

    # Rate Limiting
    rate_limit_chat: int = int(os.getenv("RATE_LIMIT_CHAT", "100"))
    rate_limit_translate: int = int(os.getenv("RATE_LIMIT_TRANSLATE", "50"))
    rate_limit_agents: int = int(os.getenv("RATE_LIMIT_AGENTS", "20"))

    # Redis/Cache (optional)
    redis_url: str = os.getenv("REDIS_URL", "redis://localhost:6379")
    cache_ttl_chat: int = int(os.getenv("CACHE_TTL_CHAT", "3600"))
    cache_ttl_translations: int = int(os.getenv("CACHE_TTL_TRANSLATIONS", "1209600"))

    class Config:
        env_file = ".env"
        case_sensitive = False


# Create global settings instance
settings = Settings()


def validate_settings() -> tuple[bool, list[str]]:
    """
    Validate that all required settings are configured
    Returns: (is_valid, list_of_errors)
    """
    errors = []

    if not settings.database_url and not (settings.db_user and settings.db_password):
        errors.append("DATABASE_URL or DB credentials (DB_USER, DB_PASSWORD) required")

    if not settings.openai_api_key:
        errors.append("OPENAI_API_KEY is required")

    if not settings.jwt_secret or settings.jwt_secret == "your-jwt-secret-change-in-production":
        errors.append("JWT_SECRET must be set and changed from default")

    return len(errors) == 0, errors
