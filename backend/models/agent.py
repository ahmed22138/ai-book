"""
Subagent invocation logging model
"""
from sqlalchemy import Column, String, Text, Integer, Float, DateTime, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class SubagentInvocation(Base):
    """Log of AI subagent invocations"""

    __tablename__ = "subagent_invocations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="SET NULL"), nullable=True)
    agent_name = Column(String(255), nullable=False, index=True)
    input_payload = Column(JSONB, nullable=False)
    output = Column(Text, nullable=False)
    execution_time_ms = Column(Integer, nullable=False)
    tokens_input = Column(Integer, nullable=True)
    tokens_output = Column(Integer, nullable=True)
    cost_usd = Column(Float, nullable=True)
    status = Column(String(50), nullable=False, default="success")  # 'success', 'error', 'timeout'
    error_message = Column(String(1000), nullable=True)
    created_at = Column(DateTime, nullable=False, server_default=func.now(), index=True)

    def __repr__(self):
        return f"<SubagentInvocation(agent={self.agent_name}, status={self.status}, time={self.execution_time_ms}ms)>"
