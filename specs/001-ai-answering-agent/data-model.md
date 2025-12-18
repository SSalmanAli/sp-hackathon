# Data Model: AI Answering Agent for Physical AI Book

**Feature**: AI Answering Agent for Physical AI Book
**Date**: 2025-12-17
**Branch**: 001-ai-answering-agent

## Overview

This document defines the data models for the AI Answering Agent that operates within the Physical AI Book chatbot. The models are designed to support tool-augmented reasoning while maintaining the separation of concerns required by the constitutional principles.

## Core Entities

### UserQuery Model
**Purpose**: Represents a user's natural language question submitted to the AI Answering Agent

**Fields**:
- `query_text` (string, required): The user's natural language question
- `user_context` (object, optional): Additional context about the user or conversation history
- `timestamp` (datetime, required): When the query was submitted
- `query_id` (string, required): Unique identifier for the query

**Validation Rules**:
- `query_text` must be 1-1000 characters
- `query_id` must be unique within the system
- `timestamp` must be in ISO 8601 format

### ContentExtractionToolRequest Model
**Purpose**: Represents a request to the Content Extraction tool for relevant book content

**Fields**:
- `query_text` (string, required): The original query text to search for context
- `context_requirements` (object, optional): Specific types of context needed (e.g., modules, lessons, sections)
- `max_results` (integer, optional): Maximum number of content chunks to return (default: 5)
- `similarity_threshold` (float, optional): Minimum similarity threshold for results (default: 0.7)

**Validation Rules**:
- `query_text` must match the original user query
- `max_results` must be between 1 and 20
- `similarity_threshold` must be between 0.0 and 1.0

### ContentExtractionToolResponse Model
**Purpose**: Represents the response from the Content Extraction tool with relevant book content

**Fields**:
- `content_chunks` (array of ContentChunk, required): Array of relevant content chunks from the book
- `total_chunks_found` (integer, required): Total number of relevant chunks found
- `extraction_time_ms` (float, required): Time taken to extract content in milliseconds
- `query_id` (string, required): Reference to the original query ID

**Validation Rules**:
- `content_chunks` array cannot be empty if relevant content exists
- `total_chunks_found` must be >= length of `content_chunks` array
- `extraction_time_ms` must be >= 0

### ContentChunk Model
**Purpose**: Represents a segment of Physical AI Book content extracted by the Content Extraction tool

**Fields**:
- `chunk_id` (string, required): Unique identifier for the content chunk
- `module_id` (string, required): Module identifier where this content belongs
- `lesson_id` (string, required): Lesson identifier within the module
- `section_id` (string, required): Section identifier within the lesson
- `content` (string, required): The actual text content of the chunk
- `similarity_score` (float, required): Similarity score between query and content chunk
- `metadata` (object): Additional metadata about the content

**Validation Rules**:
- All ID fields must be non-empty strings
- `content` must be 10-5000 characters
- `similarity_score` must be between 0.0 and 1.0

### AIResponse Model
**Purpose**: Represents the AI assistant's answer to the user, grounded in extracted content and formatted for chatbot UI

**Fields**:
- `response_text` (string, required): The AI's response to the user's query
- `source_references` (array of ContextReference, required): References to book content that support the response
- `confidence_level` (string, required): Confidence level of the response (high, medium, low)
- `response_time_ms` (float, required): Time taken to generate response in milliseconds
- `query_id` (string, required): Reference to the original query ID
- `is_information_available` (boolean, required): Whether the requested information was found in the book

**Validation Rules**:
- `response_text` must be provided and non-empty when `is_information_available` is true
- `source_references` must not be empty when `is_information_available` is true
- `confidence_level` must be one of: "high", "medium", "low"
- `response_time_ms` must be >= 0

### ContextReference Model
**Purpose**: Specific references to book content (module, lesson, section, chunk identifiers) that support the AI response

**Fields**:
- `chunk_id` (string, required): Reference to the specific content chunk
- `module` (string, required): Module name where content is found
- `lesson` (string, required): Lesson name where content is found
- `section` (string, required): Section name where content is found
- `relevance_score` (float, required): How relevant this content is to the query

**Validation Rules**:
- All reference fields must match existing content in the book
- `relevance_score` must be between 0.0 and 1.0

## Relationships

```
UserQuery --[1-to-1]--> ContentExtractionToolRequest --[1-to-1]--> ContentExtractionToolResponse --[1-to-many]--> ContentChunk
ContentExtractionToolResponse --[1-to-1]--> AIResponse --[1-to-many]--> ContextReference --[1-to-1]--> ContentChunk
```

## State Transitions

### Query Processing Flow
1. **Input State**: UserQuery received with natural language question
2. **Tool Request State**: ContentExtractionToolRequest created and sent to extraction tool
3. **Tool Response State**: ContentExtractionToolResponse received with relevant content chunks
4. **AI Reasoning State**: AI agent reasons over provided content chunks
5. **Response State**: AIResponse created with grounded answer and source references
6. **Output State**: Response returned to user with proper formatting

## Data Validation

### Input Validation
- Query text length: 1-1000 characters
- Query ID: Must be unique
- Timestamp: Must be in valid ISO 8601 format

### Processing Validation
- Content chunk similarity scores: Must be in 0.0-1.0 range
- Confidence levels: Must be "high", "medium", or "low"
- Source references: Must correspond to actual content chunks

### Output Validation
- Response text: Must be grounded in provided content
- Source references: Must not be empty when information is available
- Information availability flag: Must match whether content was found