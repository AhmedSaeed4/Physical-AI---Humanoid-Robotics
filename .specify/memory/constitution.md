<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.0.0
Modified principles:
- [PRINCIPLE_1_NAME] → I. Service Isolation & Distinct Runtimes
- [PRINCIPLE_2_NAME] → II. Guardrailed Intelligence
- [PRINCIPLE_3_NAME] → III. Profile-Driven Personalization
- [PRINCIPLE_4_NAME] → IV. Truth in Markdown (Vector Synchronization)
- [PRINCIPLE_5_NAME] → V. Secure Identity Propagation
- [SECTION_2_NAME] → Technology Standards
- [SECTION_3_NAME] → Development Workflow
- [GOVERNANCE_RULES] → Updated governance rules
Added sections: None
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending
Follow-up TODOs: None
-->

# Docusaurus RAG Chatbot Constitution

## Core Principles

### I. Service Isolation & Distinct Runtimes
The architecture is strictly tri-fold: **Frontend (Docusaurus/React)**, **Backend (FastAPI/Python)**, and **Auth Server (Node/Express)**. These must remain decoupled.
*   **Rule:** No direct database access from the Frontend.
*   **Rule:** The Frontend serves static content; dynamic features (Chat, Auth) must occur via REST API calls.
*   **Rule:** Cross-Origin Resource Sharing (CORS) must be explicitly configured to allow communication between ports 3000, 3001, and 8000 (and production equivalents).

### II. Guardrailed Intelligence
AI interactions must never directly expose the raw LLM to the user without mediation.
*   **Rule:** All chat requests must pass through the `openai-agents` Guardrail layer.
*   **Rule:** Every query must be evaluated for **Content Safety** (Pornography/Politics) and **Topic Relevance** (Book-related vs. General) before generation.
*   **Rule:** RAG retrieval (Qdrant) must occur *before* LLM generation to ground the response in factual data.

### III. Profile-Driven Personalization
The chatbot is not generic; it is a tutor that adapts to the user.
*   **Rule:** User background data (Education, Software/Hardware exp) collected during Auth **must** be injected into the System Prompt for every RAG query.
*   **Rule:** If a user is unauthenticated, the system must default to a "General/Beginner" profile but still function for basic queries (if allowed) or prompt for login.

### IV. Truth in Markdown (Vector Synchronization)
The source of truth for knowledge is the `.md` files in the `docs/` directory.
*   **Rule:** The Qdrant vector database is a derivative of the Markdown files.
*   **Rule:** Any content update to documentation requires running the `ingest.py` script to maintain synchronization between the static site and the AI brain.
*   **Rule:** Chunking strategies must preserve context (approx. 1000 chars) to ensure retrieval quality.

### V. Secure Identity Propagation
Authentication and Authorization are handled centrally but verified locally.
*   **Rule:** The **Auth Server** (Better Auth) is the single source of truth for identity.
*   **Rule:** The **Backend** must validate JWTs (from Headers or Cookies) independently before processing chat requests.
*   **Rule:** Secrets (API Keys, DB URLs) must strictly reside in `.env` files and never be committed to version control.

## Technology Standards

### Stack Requirements
*   **Frontend:** Docusaurus 3.x, React 19, TypeScript 5.x.
*   **Backend:** Python 3.12+ (managed via `uv`), FastAPI.
*   **Auth:** Node.js 20+, Better Auth, Express.
*   **Databases:** PostgreSQL (User/Chat History), Qdrant (Vectors).
*   **AI Provider:** Google Gemini (via OpenAI-compatible adapter).

### Directory & Structure
*   `backend/`: Python logic, Ingestion scripts, API.
*   `auth-server/`: Node.js Better Auth implementation.
*   `test-docs/`: Docusaurus site (Frontend).
*   Migrations must be stored in their respective service directories (e.g., `auth-server/migrations/`).

## Development Workflow

### Local Development
1.  **Triple-Terminal Setup:** Developers must run Frontend (:3000), Auth (:3001), and Backend (:8000) simultaneously.
2.  **Ingestion First:** Before testing RAG, the `ingest.py` script must be run against the local `docs/` folder.
3.  **Environment Sync:** `.env` files must be present in all three service roots based on `.env.example`.

### Deployment Gates
1.  **Build Check:** The Docusaurus build (`npm run build`) must pass without broken links (`onBrokenLinks: 'throw'`).
2.  **Linting:** TypeScript types must check out in both Frontend and Auth server.
3.  **Migration Check:** Database schema changes must be applied via SQL scripts before code deployment.

## Governance

### Amendment Process
This Constitution defines the architectural integrity of the RAG Chatbot.
*   Changes to the **Database Schema** (Postgres or Qdrant) require updating Section 8 of the Specifications.
*   Changes to the **API Contract** (between Frontend and Backend) require updating Section 9 of the Specifications.

### Compliance
*   Pull Requests must verify that no "Direct to LLM" calls exist; all must route through the RAG/Guardrail pipeline.
*   Frontend components must treat `apiUrl` as a prop/env variable to ensure compatibility with GitHub Pages (where the backend URL differs from localhost).

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-06