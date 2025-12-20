from typing import Optional
from fastapi import HTTPException, status


class RAGBaseException(Exception):
    """Base exception class for RAG agent errors"""
    def __init__(self, message: str, error_code: Optional[str] = None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class RetrievalError(RAGBaseException):
    """Exception raised when retrieval fails"""
    pass


class GenerationError(RAGBaseException):
    """Exception raised when response generation fails"""
    pass


class ConfigurationError(RAGBaseException):
    """Exception raised when configuration is invalid"""
    pass


class ValidationError(RAGBaseException):
    """Exception raised when input validation fails"""
    pass


class APIConnectionError(RAGBaseException):
    """Exception raised when API connection fails"""
    pass


def create_http_exception(
    status_code: int,
    detail: str,
    headers: Optional[dict] = None
) -> HTTPException:
    """
    Create an HTTP exception with the specified status code and detail.

    Args:
        status_code: HTTP status code
        detail: Error detail message
        headers: Optional headers to include

    Returns:
        HTTPException instance
    """
    return HTTPException(
        status_code=status_code,
        detail=detail,
        headers=headers
    )


def handle_retrieval_error(error: Exception, query: str) -> HTTPException:
    """
    Handle retrieval errors and return appropriate HTTP exception.

    Args:
        error: The original error
        query: The query that caused the error

    Returns:
        HTTPException instance
    """
    error_msg = f"Failed to retrieve relevant content for query: {query}. Error: {str(error)}"
    return create_http_exception(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=error_msg
    )


def handle_generation_error(error: Exception, context: str) -> HTTPException:
    """
    Handle generation errors and return appropriate HTTP exception.

    Args:
        error: The original error
        context: The context that was used for generation

    Returns:
        HTTPException instance
    """
    error_msg = f"Failed to generate response with context. Error: {str(error)}"
    return create_http_exception(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=error_msg
    )


def handle_validation_error(error: Exception, field: str) -> HTTPException:
    """
    Handle validation errors and return appropriate HTTP exception.

    Args:
        error: The original error
        field: The field that failed validation

    Returns:
        HTTPException instance
    """
    error_msg = f"Validation failed for field '{field}': {str(error)}"
    return create_http_exception(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=error_msg
    )