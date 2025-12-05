"""
Chat message model for RAG chatbot
"""
from sqlalchemy import Column, String, Text, Float, Integer, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class ChatMessage(Base):
    """Chat message history with RAG responses"""

    __tablename__ = "chat_messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="SET NULL"), nullable=True)
    query = Column(Text, nullable=False)
    selected_text = Column(Text, nullable=True)
    chapter = Column(String(255), nullable=True)
    response = Column(Text, nullable=False)
    sources = Column(JSONB, nullable=False, default=list)
    confidence = Column(Float, nullable=False)  # 0.0 - 1.0
    response_time_ms = Column(Integer, nullable=False)
    tokens_used = Column(Integer, nullable=True)
    feedback = Column(String(50), nullable=True)  # 'helpful', 'not_helpful', etc.
    created_at = Column(DateTime, nullable=False, server_default=func.now(), index=True)

    def __repr__(self):
        return f"<ChatMessage(id={self.id}, user_id={self.user_id}, confidence={self.confidence})>"
