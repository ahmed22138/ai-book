"""
User and Profile database models
"""
from sqlalchemy import Column, String, Boolean, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, ARRAY
from sqlalchemy.orm import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class User(Base):
    """User account model"""

    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    name = Column(String(255), nullable=False)
    created_at = Column(DateTime, nullable=False, server_default=func.now())
    updated_at = Column(DateTime, nullable=False, server_default=func.now(), onupdate=func.now())
    last_login = Column(DateTime, nullable=True)

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email}, name={self.name})>"


class Profile(Base):
    """User profile and personalization settings"""

    __tablename__ = "profiles"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), unique=True, nullable=False)
    software_experience = Column(String(50), nullable=False, default="Beginner")
    hardware_experience = Column(Boolean, nullable=False, default=False)
    personalization_level = Column(String(50), nullable=False, default="Beginner")
    preferred_language = Column(String(10), nullable=False, default="en")
    completed_chapters = Column(ARRAY(String), nullable=False, default=list)
    created_at = Column(DateTime, nullable=False, server_default=func.now())
    updated_at = Column(DateTime, nullable=False, server_default=func.now(), onupdate=func.now())

    def __repr__(self):
        return f"<Profile(user_id={self.user_id}, software_exp={self.software_experience})>"
