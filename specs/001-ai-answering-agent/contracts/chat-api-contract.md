# OpenAPI Contract: Chat API for AI Answering Agent

**Feature**: AI Answering Agent for Physical AI Book
**Date**: 2025-12-17
**Branch**: 001-ai-answering-agent

## Overview

This document specifies the OpenAPI contract for the chat API that enables the AI Answering Agent to respond to user queries within the Physical AI Book chatbot.

## OpenAPI Specification

```yaml
openapi: 3.0.3
info:
  title: Physical AI Book Chat API
  description: API for AI Answering Agent that responds to user queries within the Physical AI Book chatbot
  version: 1.0.0
  contact:
    name: Physical AI Book Team

servers:
  - url: https://api.physical-ai-book.com/v1
    description: Production server
  - url: https://staging-api.physical-ai-book.com/v1
    description: Staging server

paths:
  /chat:
    post:
      summary: Process user query and return AI-generated response
      description: Accepts a user query and returns an AI-generated response grounded in Physical AI Book content
      operationId: processUserQuery
      tags:
        - chat
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ChatRequest'
            example:
              query: "How does a PID controller work in robotics?"
              user_context:
                conversation_history: []
                user_preferences: {}
      responses:
        '200':
          description: Successful response from AI Answering Agent
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ChatResponse'
              example:
                query: "How does a PID controller work in robotics?"
                response: "A PID (Proportional-Integral-Derivative) controller in robotics is a control loop mechanism that calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error by adjusting the process control inputs. The PID controller uses three terms: Proportional (P) which reduces the rise time and reduces, but never eliminates, the steady-state error; Integral (I) which eliminates the steady-state error but may worsen the transient response; and Derivative (D) which increases the stability of the system, reduces overshoot, and improves the transient response."
                source_references:
                  - chunk_id: "chunk-001"
                    module: "Robot Control Systems"
                    lesson: "Motor Control Algorithms"
                    section: "PID Controllers"
                    relevance_score: 0.92
                  - chunk_id: "chunk-002"
                    module: "Control Theory"
                    lesson: "Feedback Control"
                    section: "PID Fundamentals"
                    relevance_score: 0.87
                confidence_level: "high"
                is_information_available: true
                response_time_ms: 1250.5
                query_id: "query-12345"
        '400':
          description: Bad request - invalid query parameters
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '500':
          description: Internal server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
      security:
        - api_key: []

  /chat/health:
    get:
      summary: Health check for the AI Answering Agent
      description: Check if the AI Answering Agent is operational
      operationId: chatHealthCheck
      tags:
        - chat
      responses:
        '200':
          description: AI Answering Agent is healthy
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HealthResponse'
              example:
                status: "healthy"
                service: "ai-answering-agent"
                timestamp: "2025-12-17T10:00:00Z"

components:
  schemas:
    ChatRequest:
      type: object
      required:
        - query
      properties:
        query:
          type: string
          description: The user's natural language question
          example: "How does a PID controller work in robotics?"
          minLength: 1
          maxLength: 1000
        user_context:
          type: object
          description: Additional context about the user or conversation
          properties:
            conversation_history:
              type: array
              items:
                type: object
                properties:
                  role:
                    type: string
                    enum: [user, assistant]
                  content:
                    type: string
              description: Previous conversation turns
            user_preferences:
              type: object
              description: User preferences that may affect response

    ChatResponse:
      type: object
      required:
        - query
        - response
        - source_references
        - confidence_level
        - is_information_available
        - response_time_ms
        - query_id
      properties:
        query:
          type: string
          description: The original user query
          example: "How does a PID controller work in robotics?"
        response:
          type: string
          description: The AI-generated response grounded in book content
          example: "A PID (Proportional-Integral-Derivative) controller in robotics is a control loop mechanism..."
        source_references:
          type: array
          items:
            $ref: '#/components/schemas/ContextReference'
          description: References to book content that support the response
        confidence_level:
          type: string
          description: Confidence level of the response
          enum: [high, medium, low]
          example: "high"
        is_information_available:
          type: boolean
          description: Whether the requested information was found in the book
          example: true
        response_time_ms:
          type: number
          description: Time taken to generate the response in milliseconds
          example: 1250.5
        query_id:
          type: string
          description: Unique identifier for the query
          example: "query-12345"

    ContextReference:
      type: object
      required:
        - chunk_id
        - module
        - lesson
        - section
        - relevance_score
      properties:
        chunk_id:
          type: string
          description: Unique identifier for the content chunk
          example: "chunk-001"
        module:
          type: string
          description: Module name where content is found
          example: "Robot Control Systems"
        lesson:
          type: string
          description: Lesson name where content is found
          example: "Motor Control Algorithms"
        section:
          type: string
          description: Section name where content is found
          example: "PID Controllers"
        relevance_score:
          type: number
          description: How relevant this content is to the query
          minimum: 0.0
          maximum: 1.0
          example: 0.92

    ErrorResponse:
      type: object
      required:
        - error
        - message
      properties:
        error:
          type: string
          description: Error code
          example: "QUERY_PROCESSING_ERROR"
        message:
          type: string
          description: Human-readable error message
          example: "Error processing user query"

    HealthResponse:
      type: object
      required:
        - status
        - service
        - timestamp
      properties:
        status:
          type: string
          description: Health status
          example: "healthy"
        service:
          type: string
          description: Name of the service
          example: "ai-answering-agent"
        timestamp:
          type: string
          format: date-time
          description: Timestamp of the health check
          example: "2025-12-17T10:00:00Z"

  securitySchemes:
    api_key:
      type: apiKey
      name: X-API-Key
      in: header
      description: API key for authentication
```

## Contract Validation Requirements

1. All request parameters must be validated according to schema definitions
2. Response bodies must conform to the specified schemas
3. HTTP status codes must match the documented responses
4. Error responses must follow the ErrorResponse schema
5. API must authenticate using the specified API key mechanism
6. All responses must be grounded in tool-provided context without hallucination
7. When information is not available, responses must explicitly state this