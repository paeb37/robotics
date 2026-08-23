# Multi-stage: the C++ solver is compiled in one image and only its shared
# library is carried into the runtime image. Eigen, OSQP and a compiler exist
# only at build time, so the runtime never ships a toolchain.
#
# Ubuntu 24.04 is not arbitrary -- it is one of the distributions Drake's pip
# wheels support. See https://drake.mit.edu/installation.html

# ---------------------------------------------------------------- builder ---
FROM ubuntu:24.04 AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ca-certificates \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
# Only the C++ sources: changing Python code must not invalidate this layer.
COPY cpp/ cpp/
RUN cmake -S cpp -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel

# ---------------------------------------------------------------- runtime ---
FROM ubuntu:24.04

# libx11/libsm/libglib are Drake's runtime dependencies on Ubuntu 24.04. They
# are not optional even headless: pydrake fails to import without them.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-venv \
        ca-certificates \
        libx11-6 \
        libsm6 \
        libglib2.0-0t64 \
    && rm -rf /var/lib/apt/lists/*

# A venv rather than the system interpreter: Ubuntu 24.04 marks its Python as
# externally managed (PEP 668), and this avoids fighting that.
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv "$VIRTUAL_ENV"
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

WORKDIR /app

# Dependencies first, so source edits do not re-download the Drake wheel.
COPY pyproject.toml ./
COPY cartpole/__init__.py cartpole/
RUN pip install --no-cache-dir -e .

COPY cartpole/ cartpole/
COPY scripts/ scripts/
COPY tests/ tests/

# cpp_backend.py looks for the library at <repo root>/build/
COPY --from=builder /src/build/libcartpole_mpc.so build/

# Smoke test at build time, so a broken image never ships.
#
# This deliberately solves a QP through BOTH backends and compares them. An
# earlier version only imported cartpole.cpp_backend, which touches nothing but
# ctypes and numpy -- so it passed while pydrake was in fact unimportable for
# want of libglib. An assertion that cannot fail is worse than no assertion.
RUN python -c "\
import numpy as np, pydrake.all; \
from cartpole.mpc import make_mpc_controller; \
e = np.array([0.0, 0.05, 0.0, 0.0]); \
a = make_mpc_controller(backend='python').solve_qp(e); \
b = make_mpc_controller(backend='cpp').solve_qp(e); \
assert a is not None and b is not None, 'solve failed'; \
assert abs(a - b) < 1e-6, (a, b); \
print(f'smoke OK: pydrake imports, both backends agree ({a:.6f})')"

CMD ["python", "scripts/bench_mpc.py"]
