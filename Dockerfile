FROM python:3.12-slim
WORKDIR /workspace
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 PYTHONPATH=/workspace/src
COPY src/ /workspace/src/
COPY tests/ /workspace/tests/
COPY simulation/ /workspace/simulation/
COPY semantic_data/ /workspace/semantic_data/
COPY scripts/ /workspace/scripts/
RUN chmod +x /workspace/scripts/*.sh
ENTRYPOINT ["python","-m","autonomymind_runtime.cli"]
