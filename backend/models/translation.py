"""
Translation cache model
"""
from sqlalchemy import Column, String, Text, DateTime
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class Translation(Base):
    """Cached translations of chapters"""

    __tablename__ = "translations"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    chapter_id = Column(String(255), nullable=False, index=True)
    language = Column(String(10), nullable=False)
    content = Column(Text, nullable=False)
    source_language = Column(String(10), nullable=False, default="en")
    translation_model = Column(String(50), nullable=False)
    created_at = Column(DateTime, nullable=False, server_default=func.now())
    expires_at = Column(DateTime, nullable=True)
    metadata = Column(JSONB, nullable=True)

    __table_args__ = (
        # Composite unique constraint
        # This will be added via migration or raw SQL
    )

    def __repr__(self):
        return f"<Translation(chapter_id={self.chapter_id}, language={self.language})>"
