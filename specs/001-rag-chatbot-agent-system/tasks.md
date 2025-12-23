# Tasks: RAG Chatbot with Agent Architecture and Ephemeral Sessions

**Input**: Design documents from `/specs/001-rag-chatbot-agent-system/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Tests are NOT included in this task list as they were not explicitly requested in the feature specification. Focus is on implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `RAG-backend/app/` for source, `RAG-backend/tests/` for tests
- **Frontend**: `frontend/Physical AI and Robotics/src/` for source

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Verify Python 3.11+ and Node 20+ installed per quickstart.md prerequisites
- [ ] T002 Create backend directory structure per plan.md (agents/, api/, models/, services/, db/, utils/)
- [ ] T003 [P] Install backend dependencies from RAG-backend/requirements.txt
- [ ] T004 [P] Install frontend dependencies and add Zustand to frontend/Physical AI and Robotics/package.json
- [ ] T005 [P] Configure environment variables: copy RAG-backend/.env.example to .env and fill in OPENAI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
- [ ] T006 [P] Setup Jest configuration in frontend/Physical AI and Robotics/jest.config.js per research.md
- [ ] T007 [P] Configure CORS in RAG-backend/app/main.py to allow localhost:3000 and GitHub Pages origin

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T008 Create Pydantic base models in RAG-backend/app/models/__init__.py
- [ ] T009 [P] Create Message model in RAG-backend/app/models/message.py per data-model.md spec
- [ ] T010 [P] Create QueryResult model with RetrievedChunk and ChunkMetadata in RAG-backend/app/models/query_result.py
- [ ] T011 [P] Create AgentResponse model with AgentName enum in RAG-backend/app/models/agent_response.py
- [ ] T012 [P] Create ErrorContext model with ErrorType enum in RAG-backend/app/models/error_context.py
- [ ] T013 Create Qdrant client wrapper in RAG-backend/app/db/qdrant_client.py with connection management
- [ ] T014 [P] Create embedding service in RAG-backend/app/services/embedding.py (Cohere/OpenAI integration)
- [ ] T015 [P] Create LLM service in RAG-backend/app/services/llm.py (OpenAI integration)
- [ ] T016 [P] Create Qdrant service in RAG-backend/app/services/qdrant.py wrapping qdrant_client
- [ ] T017 Setup structured logging in RAG-backend/app/utils/logging.py using structlog
- [ ] T018 Create FastAPI app instance in RAG-backend/app/main.py with CORS middleware
- [ ] T019 [P] Create Zustand chat store in frontend/Physical AI and Robotics/src/store/chatStore.ts per research.md implementation
- [ ] T020 [P] Create TypeScript types in frontend/Physical AI and Robotics/src/types/message.ts and api.ts from contracts/types.ts
- [ ] T021 Create API client in frontend/Physical AI and Robotics/src/services/chatApi.ts with environment-based BASE_URL

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Documentation Questions (Priority: P1) 🎯 MVP

**Goal**: Users can ask questions about documentation and receive accurate answers grounded only in the documentation, with source attribution

**Independent Test**: Open chatbot, ask "What is ROS2?", verify answer contains only documentation content with chapter/section references

### Implementation for User Story 1

#### Backend Agents (Core RAG Pipeline)

- [ ] T022 [P] [US1] Create QueryRouterAgent in RAG-backend/app/agents/query_router.py that validates input, detects greetings/empty queries, normalizes text (FR-005)
- [ ] T023 [P] [US1] Create QdrantRetrievalAgent in RAG-backend/app/agents/qdrant_retrieval.py that embeds query, retrieves top-K chunks, handles Qdrant failures (FR-006)
- [ ] T024 [P] [US1] Create system prompt file in RAG-backend/app/utils/prompts/answer_synthesis.txt that prohibits hallucination and external knowledge (FR-009)
- [ ] T025 [P] [US1] Create AnswerSynthesisAgent in RAG-backend/app/agents/answer_synthesis.py that generates answers using ONLY retrieved context and loads system prompt (FR-007)
- [ ] T026 [P] [US1] Create ErrorRecoveryAgent in RAG-backend/app/agents/error_recovery.py that catches errors, classifies them, and converts to user-safe messages (FR-008)
- [ ] T027 [US1] Create RAGChatAgent orchestrator in RAG-backend/app/agents/rag_chat_agent.py that coordinates all sub-agents using OpenAI Agents SDK (FR-004)

#### Backend API Endpoint

- [ ] T028 [US1] Create query endpoint in RAG-backend/app/api/query.py that accepts QueryRequest and returns QueryResponse via RAGChatAgent (FR-002)
- [ ] T029 [US1] Add query router to main.py FastAPI app
- [ ] T030 [US1] Add request/response logging to query endpoint with processing time tracking (SC-001 monitoring)

#### Frontend Components

- [ ] T031 [P] [US1] Create MessageBubble component in frontend/Physical AI and Robotics/src/components/MessageBubble.tsx that displays individual messages with role-based styling
- [ ] T032 [P] [US1] Create ChatInput component in frontend/Physical AI and Robotics/src/components/ChatInput.tsx with loading state, max length validation (10,000 chars), and send button
- [ ] T033 [US1] Create ChatWindow component in frontend/Physical AI and Robotics/src/components/ChatWindow.tsx that renders message list, handles scrolling, and integrates ChatInput
- [ ] T034 [US1] Create FloatingChatWidget component in frontend/Physical AI and Robotics/src/components/FloatingChatWidget.tsx with bottom-left positioning, high z-index, open/close toggle (FR-013, FR-014)
- [ ] T035 [US1] Wire ChatWindow to Zustand store (useChatStore) for message state and send message action
- [ ] T036 [US1] Implement query API call in chatApi.ts service and wire to send message action in store
- [ ] T037 [US1] Add loading indicator in ChatInput while query is processing (FR-019)
- [ ] T038 [US1] Display source attribution (chapter/section) in MessageBubble for assistant messages (SC-007)

#### Integration & Styling

- [ ] T039 [US1] Create chat widget CSS in frontend/Physical AI and Robotics/src/css/chat-widget.css for positioning, z-index (9999), and animations
- [ ] T040 [US1] Integrate FloatingChatWidget into Docusaurus site (add to theme or layout component)
- [ ] T041 [US1] Test end-to-end flow: open widget → ask question → receive answer with sources → verify no hallucination on out-of-scope question

**Checkpoint**: At this point, User Story 1 should be fully functional - users can ask questions and get document-grounded answers

---

## Phase 4: User Story 2 - Ephemeral Chat Sessions (Priority: P2)

**Goal**: Chat history is completely cleared when the chatbot is closed, ensuring privacy and fresh sessions

**Independent Test**: Open chatbot, ask 3 questions, close widget, reopen widget, verify chat is empty with no previous messages

### Implementation for User Story 2

- [ ] T042 [US2] Implement closeChat action in frontend/Physical AI and Robotics/src/store/chatStore.ts that sets isChatOpen=false AND clears messages array (FR-016)
- [ ] T043 [US2] Wire close button in FloatingChatWidget to call useChatStore closeChat action
- [ ] T044 [US2] Add useEffect in ChatWindow that clears messages when isChatOpen transitions to false (< 100ms per SC-003)
- [ ] T045 [US2] Verify no localStorage, sessionStorage, or cookie usage for chat messages in chatStore.ts (FR-017, FR-021-024)
- [ ] T046 [US2] Add performance logging to measure closeChat execution time (validate SC-003: <100ms)
- [ ] T047 [US2] Test page refresh clears chat history (no cross-page persistence)
- [ ] T048 [US2] Verify backend has no session storage for chat messages (confirm stateless per FR-022-023)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - users get answers AND sessions are ephemeral

---

## Phase 5: User Story 3 - Graceful Error Handling (Priority: P3)

**Goal**: Clear, friendly error messages when something goes wrong, with no technical details exposed to users

**Independent Test**: Simulate Qdrant unavailable, ask question, verify friendly error message displayed (not stack trace), verify backend logs full error

### Implementation for User Story 3

#### Backend Error Handling

- [ ] T049 [P] [US3] Enhance ErrorRecoveryAgent in RAG-backend/app/agents/error_recovery.py to classify all error types (network, database, llm, validation, agent, unknown)
- [ ] T050 [P] [US3] Add error sanitization function in ErrorRecoveryAgent that removes stack traces and sensitive data from user_safe_message (SC-005)
- [ ] T051 [US3] Add server-side error logging with full stack traces in ErrorRecoveryAgent (FR-011)
- [ ] T052 [US3] Update RAGChatAgent to catch all exceptions and route through ErrorRecoveryAgent
- [ ] T053 [US3] Update query endpoint to return ErrorResponse (400/500) with sanitized error messages
- [ ] T054 [US3] Add request_id generation in query endpoint for error tracking

#### Frontend Error Display

- [ ] T055 [P] [US3] Create SystemNotification component in frontend/Physical AI and Robotics/src/components/SystemNotification.tsx that displays error with dismiss and retry buttons (FR-018, FR-020)
- [ ] T056 [US3] Add error state to Zustand store (chatStore.ts) with setError action
- [ ] T057 [US3] Update chatApi.ts to catch fetch errors and convert to ApiError type with user-safe messages
- [ ] T058 [US3] Wire query API errors to setError in store and display SystemNotification in ChatWindow
- [ ] T059 [US3] Implement retry logic in SystemNotification that re-calls query endpoint
- [ ] T060 [US3] Add error message mapping in frontend (use ERROR_MESSAGES from contracts/types.ts)

#### Edge Case Handling

- [ ] T061 [US3] Add greeting detection in QueryRouterAgent that returns friendly response without retrieval (edge case: "Hello")
- [ ] T062 [US3] Add input length validation in QueryRouterAgent (reject >10,000 chars with clear message)
- [ ] T063 [US3] Handle Qdrant empty results in QdrantRetrievalAgent (return explicit "information not available" message)
- [ ] T064 [US3] Add timeout handling in LLM service with user-friendly timeout message
- [ ] T065 [US3] Test error scenarios: Qdrant down, LLM timeout, empty query, oversized query, malformed input

**Checkpoint**: All user stories should now be independently functional with robust error handling

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Additional endpoints, monitoring, and deployment readiness

### Additional API Endpoints

- [ ] T066 [P] Create health endpoint in RAG-backend/app/api/health.py that checks Qdrant connection and LLM availability (FR-001)
- [ ] T067 [P] Create chapters endpoint in RAG-backend/app/api/chapters.py that returns documentation metadata (FR-003)
- [ ] T068 Add health and chapters routers to main.py FastAPI app

### Performance & Monitoring

- [ ] T069 [P] Add performance tracking to query endpoint (measure total processing time, Qdrant retrieval time, LLM generation time)
- [ ] T070 [P] Add metrics logging for SC-001 (<3s for 95% queries) and SC-008 (50 concurrent queries)
- [ ] T071 [P] Add confidence score calculation in AnswerSynthesisAgent based on retrieval scores
- [ ] T072 Verify chat widget visibility across desktop/tablet/mobile screen sizes (SC-009)

### Documentation & Configuration

- [ ] T073 [P] Update RAG-backend/README.md with setup instructions from quickstart.md
- [ ] T074 [P] Update RAG-backend/.env.example with all required environment variables
- [ ] T075 [P] Create RAG-backend/Dockerfile for Hugging Face Spaces deployment (SC-010)
- [ ] T076 [P] Verify frontend build for GitHub Pages deployment works (npm run build)
- [ ] T077 Document API usage in frontend/Physical AI and Robotics/src/components/API_USAGE_GUIDE.md

### Final Validation

- [ ] T078 Run quickstart.md backend setup steps and verify health endpoint returns healthy
- [ ] T079 Run quickstart.md frontend setup steps and verify widget loads
- [ ] T080 Verify all 10 success criteria from spec.md (SC-001 through SC-010)
- [ ] T081 Test all edge cases from spec.md (greetings, rate limiting, concurrent queries, empty results, multilingual, missing prompt)
- [ ] T082 Validate no hardcoded secrets in codebase (FR deployment readiness)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories - FULLY INDEPENDENT
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 components but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Enhances US1/US2 with error handling but independently testable

### Within Each User Story

**User Story 1** (T022-T041):
1. Agents can be built in parallel (T022-T026 all [P])
2. RAGChatAgent orchestrator requires all sub-agents complete (T027 depends on T022-T026)
3. API endpoint requires orchestrator (T028 depends on T027)
4. Frontend components can be built in parallel (T031-T034 all [P])
5. Integration requires components ready (T035-T041 sequential)

**User Story 2** (T042-T048):
- All tasks sequential, build on US1 components

**User Story 3** (T049-T065):
- Backend error tasks can be parallel (T049-T051 all [P])
- Frontend components can be parallel (T055, T060 [P])
- Edge cases at end (T061-T065)

### Parallel Opportunities

- **Phase 1 Setup**: T003, T004, T005, T006, T007 can all run in parallel
- **Phase 2 Foundational**: T009-T012, T014-T016, T019-T020 can run in parallel
- **Phase 3 US1 Agents**: T022-T026 can all run in parallel (different agent files)
- **Phase 3 US1 Frontend**: T031-T034 can all run in parallel (different components)
- **Phase 5 US3 Backend**: T049-T051 can run in parallel
- **Phase 5 US3 Frontend**: T055, T060 can run in parallel
- **Phase 6 Polish**: T066-T067, T069-T071, T073-T077 can run in parallel
- **Different user stories** can be worked on in parallel by different developers after Foundational completes

---

## Parallel Example: User Story 1 Agents

```bash
# Launch all agent implementations in parallel:
Task T022: "Create QueryRouterAgent in RAG-backend/app/agents/query_router.py"
Task T023: "Create QdrantRetrievalAgent in RAG-backend/app/agents/qdrant_retrieval.py"
Task T024: "Create system prompt in RAG-backend/app/utils/prompts/answer_synthesis.txt"
Task T025: "Create AnswerSynthesisAgent in RAG-backend/app/agents/answer_synthesis.py"
Task T026: "Create ErrorRecoveryAgent in RAG-backend/app/agents/error_recovery.py"

# Then orchestrator (depends on all above):
Task T027: "Create RAGChatAgent in RAG-backend/app/agents/rag_chat_agent.py"
```

---

## Parallel Example: User Story 1 Frontend

```bash
# Launch all UI components in parallel:
Task T031: "Create MessageBubble in frontend/src/components/MessageBubble.tsx"
Task T032: "Create ChatInput in frontend/src/components/ChatInput.tsx"
Task T033: "Create ChatWindow in frontend/src/components/ChatWindow.tsx"
Task T034: "Create FloatingChatWidget in frontend/src/components/FloatingChatWidget.tsx"

# Then integration (sequential):
Task T035: "Wire ChatWindow to Zustand store"
Task T036: "Implement query API call and wire to store"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup → Environment ready
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories) → Core models, services, store ready
3. Complete Phase 3: User Story 1 → Full Q&A functionality
4. **STOP and VALIDATE**: Test US1 independently (open chat, ask question, get answer with sources)
5. Skip Phase 4-5, go to Phase 6: Add health/chapters endpoints
6. Deploy MVP to Hugging Face Spaces + GitHub Pages

**MVP Scope**: ~41 tasks (T001-T041 + essential Phase 6 tasks)

### Incremental Delivery

1. **Foundation** (T001-T021): Setup + Foundational → Can start coding
2. **MVP** (T022-T041 + T066-T068): User Story 1 + health/chapters → Test independently → Deploy (users can ask questions!)
3. **Privacy** (T042-T048): User Story 2 → Test independently → Deploy (ephemeral sessions added)
4. **Production** (T049-T065): User Story 3 → Test independently → Deploy (error handling complete)
5. **Polish** (T069-T082): Performance, docs, validation → Final deployment

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With 3 developers after Foundational completes:

- **Developer A**: User Story 1 (T022-T041) - Backend agents + frontend components
- **Developer B**: User Story 2 (T042-T048) - Ephemeral session logic
- **Developer C**: User Story 3 (T049-T065) - Error handling infrastructure

Stories integrate at the end but are independently testable throughout.

---

## Task Summary

**Total Tasks**: 82 tasks
- **Phase 1 Setup**: 7 tasks
- **Phase 2 Foundational**: 14 tasks (BLOCKS all user stories)
- **Phase 3 US1 (P1 - MVP)**: 20 tasks
- **Phase 4 US2 (P2)**: 7 tasks
- **Phase 5 US3 (P3)**: 17 tasks
- **Phase 6 Polish**: 17 tasks

**Tasks by User Story**:
- US1 (Ask Questions): 20 tasks
- US2 (Ephemeral Sessions): 7 tasks
- US3 (Error Handling): 17 tasks
- Foundational/Polish: 38 tasks

**Parallel Opportunities**: 32 tasks marked [P] can run in parallel within their phase

**MVP Scope**: 41 tasks (Setup + Foundational + US1 + essential endpoints)

**Independent Test Criteria**:
- **US1**: Open chat, ask "What is ROS2?", verify answer with sources, ask out-of-scope question, verify refusal
- **US2**: Ask 3 questions, close widget, reopen, verify empty chat
- **US3**: Simulate errors (Qdrant down, timeout), verify friendly messages, no stack traces

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests not included (not requested in spec) - focus on implementation
- All file paths are absolute per plan.md structure
- Follow quickstart.md for development setup and debugging
- Refer to data-model.md for entity specifications
- Refer to contracts/ for API specifications
- Refer to research.md for technical decisions (Zustand, Jest, testing strategy)
