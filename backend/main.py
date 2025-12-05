"""
FastAPI application for Physical AI & Humanoid Robotics Textbook
"""
import logging
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager

from config import settings, validate_settings

# Setup logging
logging.basicConfig(
    level=settings.log_level.upper(),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for startup and shutdown"""
    # Startup
    logger.info("🚀 Starting Physical AI Textbook API...")

    # Validate settings
    is_valid, errors = validate_settings()
    if not is_valid:
        logger.error("❌ Configuration errors:")
        for error in errors:
            logger.error(f"  - {error}")
        raise RuntimeError("Configuration validation failed")

    logger.info("✅ Configuration validated")
    logger.info(f"📚 Environment: {settings.environment}")
    logger.info(f"🔗 Database: {settings.db_host}:{settings.db_port}/{settings.db_name}")
    logger.info(f"🔍 Qdrant: {settings.qdrant_url}")
    logger.info(f"🤖 OpenAI: {settings.openai_model}")

    yield

    # Shutdown
    logger.info("🛑 Shutting down Physical AI Textbook API...")


# Create FastAPI app
app = FastAPI(
    title=settings.app_name,
    description="RAG-powered AI-native textbook for Physical AI & Humanoid Robotics",
    version=settings.app_version,
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=settings.cors_credentials,
    allow_methods=settings.cors_methods,
    allow_headers=settings.cors_headers,
)


# Health check endpoint
@app.get("/health", tags=["System"])
async def health_check():
    """Check API health and service status"""
    return JSONResponse(
        status_code=200,
        content={
            "status": "healthy",
            "app": settings.app_name,
            "version": settings.app_version,
            "environment": settings.environment,
            "services": {
                "database": "configured",
                "qdrant": "configured",
                "openai": "configured",
            },
        },
    )


# Root endpoint
@app.get("/", tags=["System"])
async def root():
    """API root endpoint"""
    return {
        "message": "Physical AI & Humanoid Robotics Textbook API",
        "version": settings.app_version,
        "docs": "/docs",
        "endpoints": {
            "health": "/health",
            "chat": "/chat",
            "translate": "/translate",
            "agents": "/agent/invoke",
            "auth": "/auth/signup",
        },
    }


# Placeholder route handlers (will be implemented in subsequent phases)
@app.post("/chat", tags=["Chatbot"])
async def chat_query(query: str, selected_text: str = None, chapter: str = None):
    """RAG chatbot query endpoint (Phase 4)"""
    return {"message": "Chat endpoint - Phase 4 implementation"}


@app.post("/translate", tags=["Translation"])
async def translate_chapter(chapter_id: str, language: str):
    """Translation endpoint (Phase 7)"""
    return {"message": "Translation endpoint - Phase 7 implementation"}


@app.post("/agent/invoke", tags=["Agents"])
async def invoke_agent(agent_name: str, context: dict):
    """Subagent invocation endpoint (Phase 6)"""
    return {"message": "Agent invocation endpoint - Phase 6 implementation"}


@app.post("/auth/signup", tags=["Authentication"])
async def signup(email: str, password: str, name: str):
    """User signup endpoint (Phase 5)"""
    return {"message": "Signup endpoint - Phase 5 implementation"}


@app.post("/auth/signin", tags=["Authentication"])
async def signin(email: str, password: str):
    """User signin endpoint (Phase 5)"""
    return {"message": "Signin endpoint - Phase 5 implementation"}


# Error handlers
@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """Handle general exceptions"""
    logger.error(f"Unhandled exception: {exc}")
    return JSONResponse(
        status_code=500,
        content={"error": "Internal server error", "message": str(exc)},
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
        log_level=settings.log_level,
    )
